
#include "TopoOffsetTetMesh.h"
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "TagEnvelopes.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/bundled/format.h>
#include <wmtk/utils/predicates.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/boundary_facets.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <paraviewo/VTUWriter.hpp>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <unordered_map>


namespace wmtk::components::topological_offset {


// assumes tag has been found. won't be called otherwise
void TopoOffsetTetMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const MatrixXd& V_env,
    const MatrixXi F_env,
    const std::vector<std::string>& tag_names)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tags.rows());
    assert((V_env.rows() == 0) || (V_env.cols() == 3));
    assert((F_env.rows() == 0) || (F_env.cols() == 3));
    m_tags_count = T_tags.cols() + 1; // + 1 is for ambient

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.resize(V.rows());
    m_edge_attribute.resize(6 * T.rows());
    m_face_attribute.resize(4 * T.rows());
    m_tet_attribute.resize(T.rows());

    // set tag string/id maps
    m_tag_id_to_name[0] = "ambient";
    m_tag_name_to_id["ambient"] = 0;
    for (int64_t i = 0; i < tag_names.size(); i++) {
        m_tag_id_to_name[i + 1] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i + 1;
    }

    // add potential new tags to maps
    for (const std::string& tag : m_offset_params.offset_output_tag) {
        if (std::find(tag_names.begin(), tag_names.end(), tag) == tag_names.end()) {
            logger().warn("Tag '{}' does not exist. Adding to mesh.", tag);
            int64_t new_id = m_tag_id_to_name.size();
            m_tag_id_to_name[new_id] = tag;
            m_tag_name_to_id[tag] = new_id;
            m_tags_count++;
        }
    }

    // collect int ids for offset output
    for (const std::string& name : m_offset_params.offset_output_tag) {
        m_offset_output_tag_ids.insert(m_tag_name_to_id[name]);
    }

    // One mask bit per input tag, ambient included, in id order. Must be assigned here: once the
    // maps are complete, and before init_surfaces_and_boundaries() seeds the vertex masks from
    // the boundary faces. Tags introduced later (the band's offset tag) get no bit -- boundary
    // membership is a property of the input partition, which is also why the masks are propagated
    // rather than ever recomputed from current tet tags.
    if (m_tag_id_to_name.size() > 64) {
        log_and_throw_error(
            "Per-tag boundary envelopes support at most 64 input tags, got {}",
            m_tag_id_to_name.size());
    }
    m_tag_bit.clear();
    for (const auto& [tag_id, name] : m_tag_id_to_name) {
        const int bit = int(m_tag_bit.size());
        m_tag_bit[tag_id] = bit;
    }

    // propagate labels to tets
    const auto& tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        for (int j = 0; j < T_tags.cols(); j++) {
            if (T_tags.coeff(t_id, j) == 1) {
                m_tet_attribute[t_id].tag.insert(j + 1);
            }
        }
        if (m_tet_attribute[t_id].tag.size() == 0) { // tet is ambient
            m_tet_attribute[t_id].tag.insert(0);
        }
    }

    assert(ambient_assert());

    // set vertex positions
    const auto& verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        set_vertex_position(v_id, V.row(v_id));
    }

    init_surfaces_and_boundaries();
}

void TopoOffsetTetMesh::init_surfaces_and_boundaries()
{
    const auto faces = get_faces();
    logger().info("F = {}", faces.size());

    // tag surface faces and vertices
    //
    // The domain wall is a region boundary and must be treated as one here. A face with no
    // opposite tetrahedron bounds the ambient region against the unmeshed outside; it has no
    // second tag only because the outside is not meshed. Held in the region envelope, the wall
    // may be refined and its vertices may move within eps -- the same contract every other region
    // boundary gets. Left out, it is held instead by on_bbox_faces alone, which freezes those
    // vertices and refuses every split in the wall, so a tet resting on the wall has its longest
    // edge unsplittable and its wall vertices unmovable, and no legal operation can repair it.
    std::vector<Eigen::Vector3i> tempF;
    std::map<int64_t, std::vector<Eigen::Vector3i>> tag_faces; // per-tag boundary buckets
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        // Whose boundary this face is. Interior face: every tag on exactly one side (the
        // symmetric difference -- multi-tag tets exist, and a tag on both sides bounds nothing
        // here). Wall face: every tag of its single tet, which is how ambient's envelope comes
        // to hold the wall.
        CellTag face_tags;
        const auto t_opp = ff.switch_tetrahedron();
        if (t_opp) {
            const auto& tag0 = m_tet_attribute[ff.tid()].tag;
            const auto& tag1 = m_tet_attribute[t_opp.value().tid()].tag;
            if (tag0 == tag1) {
                continue;
            }
            std::set_symmetric_difference(
                tag0.begin(),
                tag0.end(),
                tag1.begin(),
                tag1.end(),
                std::inserter(face_tags, face_tags.begin()));
        } else {
            face_tags = m_tet_attribute[ff.tid()].tag;
        }

        m_face_attribute[ff.fid()].m_is_surface_fs = true;

        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();

        // Seed the per-vertex boundary masks and the per-tag face buckets from the same
        // classification, so the dispatch (a face is constrained by the AND of its corners'
        // masks) and the envelopes it dispatches to can never disagree about what is where.
        const uint64_t bits = tag_bits(face_tags);
        for (const size_t v : {v1, v2, v3}) m_vertex_extra[v].m_boundary_mask |= bits;
        for (const int64_t t : face_tags) {
            tag_faces[t].emplace_back(int(v1), int(v2), int(v3));
        }
        // The region flag: every face reaching here bounds a region, the ones with a tet on both
        // sides being interior boundaries and the rest the domain wall, which carries no flag
        // because vertex_is_on_region() reads it off on_bbox_faces. It does not mean "on the
        // input complex" -- it also lands on region surfaces the offset is grown through. The
        // complex itself is m_is_on_input_complex, set in mark_input_complex_vertices() once
        // label_input_complex() has run.
        if (t_opp) {
            for (const size_t v : {v1, v2, v3}) m_vertex_extra[v].m_is_on_region = true;
        }
        // The base's own flag, a different field from the two above: those say which of the
        // offset's two tracked surfaces a vertex belongs to, this says that it belongs to one at
        // all. Every surface-aware path in the shared engine gates on it.
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
        m_vertex_attribute[v3].m_is_on_surface = true;

        tempF.emplace_back(v1, v2, v3);
    }

    if (!m_envelope && !tempF.empty()) {
        logger().info("Init per-tag envelopes from tet tags");
        // build envelopes
        std::vector<Eigen::Vector3d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        m_V_envelope = tempV;
        m_F_envelope = tempF;
        // The half-width must come from the parameters, as in 2D: left at its -1 sentinel the
        // exact envelopes are built negative, so is_outside() answers true for every triangle
        // including the ones they were constructed from, freezing every boundary solid.
        //
        // params.init() runs before init_from_image(), so envelope_size is already resolved from
        // envelope_size_rel x the bbox diagonal here. Unit tests construct without it, which is
        // why this block stays behind the non-throwing tempF guard rather than throwing on a
        // nonpositive eps.
        m_envelope_eps = m_offset_params.envelope_size;

        // One exact envelope per tag, from that tag's boundary bucket -- the input partition as
        // it stands before offset construction rewrites tags, which is what makes E_t the tube
        // around the as-loaded geometry the offset potential also measures against. A boundary
        // face between two regions enters both regions' envelopes; the wall enters its tets'
        // tags'. See the m_tag_envelopes doc in the header for the intersection semantics.
        m_tag_envelopes.clear();
        {
            std::lock_guard<std::mutex> lock(m_isect_mutex);
            m_isect_cache.clear();
        }
        std::vector<std::shared_ptr<SampleEnvelope>> members;
        std::string per_tag_log;
        for (const auto& [tag, bucket] : tag_faces) {
            if (bucket.empty()) continue; // offset_output_tag ids with no tets yet
            auto env = std::make_shared<SampleEnvelope>();
            env->use_exact = true;
            env->init(m_V_envelope, bucket, m_envelope_eps);
            m_tag_envelopes[tag] = env;
            members.push_back(env);
            per_tag_log += fmt::format(" {}:{}", m_tag_id_to_name.at(tag), bucket.size());
        }

        // The base's pointer is the union of the members -- inside any tube -- because the one
        // shared-engine site that reads it directly (the collapse_edge_before point check) asks
        // exactly that. Everything else dispatches per simplex through envelope_for_mask().
        m_envelope = std::make_shared<UnionEnvelope>(std::move(members));
        logger().info(
            "\tPer-tag boundary envelopes: {} faces total (tag boundaries + domain wall), "
            "EXACT (eps {:.6g}) |{}",
            m_F_envelope.size(),
            m_envelope_eps,
            per_tag_log);
    }

    // track bounding box. box_min/box_max are only set by Parameters::init(), which callers
    // that go through the full offset pipeline call before init_from_image() -- but plenty of
    // unit tests construct a mesh directly from a default-constructed Parameters and never
    // call it, leaving these as empty (size 0) VectorXd. Skip rather than index out of bounds.
    if (m_offset_params.box_min.size() >= 3 && m_offset_params.box_max.size() >= 3) {
        for (size_t i = 0; i < faces.size(); i++) {
            const auto vs = get_face_vertices(faces[i]);
            std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            int on_bbox = -1;
            for (int k = 0; k < 3; k++) {
                if (m_vertex_attribute[vids[0]].m_posf[k] == m_offset_params.box_min[k] &&
                    m_vertex_attribute[vids[1]].m_posf[k] == m_offset_params.box_min[k] &&
                    m_vertex_attribute[vids[2]].m_posf[k] == m_offset_params.box_min[k]) {
                    on_bbox = k * 2;
                    break;
                }
                if (m_vertex_attribute[vids[0]].m_posf[k] == m_offset_params.box_max[k] &&
                    m_vertex_attribute[vids[1]].m_posf[k] == m_offset_params.box_max[k] &&
                    m_vertex_attribute[vids[2]].m_posf[k] == m_offset_params.box_max[k]) {
                    on_bbox = k * 2 + 1;
                    break;
                }
            }
            if (on_bbox < 0) {
                continue;
            }
            assert(!faces[i].switch_tetrahedron(*this)); // face must be on boundary

            const size_t fid = faces[i].fid(*this);
            m_face_attribute[fid].m_is_bbox_fs = on_bbox;

            for (const size_t vid : vids) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
            }
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });
}

bool TopoOffsetTetMesh::is_edge_on_region(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_extra[v1_id].m_is_on_region || !m_vertex_extra[v2_id].m_is_on_region)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (face_is_region(fid)) return true;
    }

    return false;
}

bool TopoOffsetTetMesh::is_edge_on_offset(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_extra[v1_id].m_is_on_offset || !m_vertex_extra[v2_id].m_is_on_offset)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (face_is_offset(fid)) return true;
    }

    return false;
}


bool TopoOffsetTetMesh::ambient_assert()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        bool has_ambient = (m_tet_attribute[t_id].tag.count(0) != 0);
        if (has_ambient && (m_tet_attribute[t_id].tag.size() != 1)) {
            std::string err_str = "";
            for (int64_t tag : m_tet_attribute[t_id].tag) {
                err_str = err_str + " " + std::to_string(tag);
            }
            logger().info(err_str);
            return false;
        }
    }
    return true;
}


void TopoOffsetTetMesh::label_input_complex()
{
    // ensure all tags exist in map
    const ExpressionPtr& expr = m_offset_params.offset_selection;
    const CellTag tags_involved = expr->tags_involved();
    for (const int64_t& tag : tags_involved) {
        auto it = m_tag_id_to_name.find(tag);
        if (it == m_tag_id_to_name.end()) {
            log_and_throw_error("Unknown tag given in offset_selection (id {})", tag);
        }
    }

    // only true if exactly one tag exists, ie "tag_0" (no &, |, !)
    bool single_body = (tags_involved.size() == 1) && expr->contains_only_or() &&
                       expr->contains_only_and() && expr->contains_only_not();

    if (single_body) {
        m_singlebody = true;
        m_single_tag = *tags_involved.begin();
        if (!(m_offset_params.offset_in || m_offset_params.offset_out)) {
            log_and_throw_error(
                "At least one of offset_in and offset_out must be true for singlebody mode.");
        }
        logger().info("Using single body mode for '{}'", m_tag_id_to_name[m_single_tag]);

        if (m_offset_params.offset_in &&
            m_offset_params.offset_out) { // input complex is body boundary
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) == 0) {
                    continue;
                }

                auto check_and_set = [this](const Tuple& tup) {
                    auto other = tup.switch_tetrahedron(*this);
                    if (!other ||
                        (m_tet_attribute[other.value().tid(*this)].tag.count(m_single_tag) == 0)) {
                        m_face_extra[tup.fid(*this)].label = 1;
                        // propagate to children simplices
                        m_edge_attribute[tup.eid(*this)].label = 1;
                        m_edge_attribute[tup.switch_edge(*this).eid(*this)].label = 1;
                        m_edge_attribute[tup.switch_vertex(*this).switch_edge(*this).eid(*this)]
                            .label = 1;
                        m_vertex_extra[tup.vid(*this)].label = 1;
                        m_vertex_extra[tup.switch_vertex(*this).vid(*this)].label = 1;
                        m_vertex_extra[tup.switch_edge(*this).switch_vertex(*this).vid(*this)]
                            .label = 1;
                    }
                };

                // for each face, check if boundary and if so, set as input
                check_and_set(t);
                check_and_set(t.switch_face(*this));
                check_and_set(t.switch_edge(*this).switch_face(*this));
                check_and_set(t.switch_vertex(*this).switch_edge(*this).switch_face(*this));
            }
        } else if (m_offset_params.offset_in) { // input complex is everything outside body
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) == 0) {
                    m_tet_attribute[t_id].label = 1;
                    // propagate to faces, edges, verts
                    auto v_ids = oriented_tet_vids(t_id);
                    for (const size_t& v_id : v_ids) {
                        m_vertex_extra[v_id].label = 1;
                    }
                    for (int i = 0; i < 6; i++) {
                        m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                    }
                    for (int i = 0; i < 4; i++) {
                        m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                    }
                } else {
                    auto check_and_set = [this](const Tuple& tup) {
                        auto other = tup.switch_tetrahedron(*this);
                        if (!other) {
                            m_face_extra[tup.fid(*this)].label = 1;
                            // propagate to children simplices
                            m_edge_attribute[tup.eid(*this)].label = 1;
                            m_edge_attribute[tup.switch_edge(*this).eid(*this)].label = 1;
                            m_edge_attribute[tup.switch_vertex(*this).switch_edge(*this).eid(*this)]
                                .label = 1;
                            m_vertex_extra[tup.vid(*this)].label = 1;
                            m_vertex_extra[tup.switch_vertex(*this).vid(*this)].label = 1;
                            m_vertex_extra[tup.switch_edge(*this).switch_vertex(*this).vid(*this)]
                                .label = 1;
                        }
                    };

                    check_and_set(t);
                    check_and_set(t.switch_face(*this));
                    check_and_set(t.switch_edge(*this).switch_face(*this));
                    check_and_set(t.switch_vertex(*this).switch_edge(*this).switch_face(*this));
                }
            }
        } else if (m_offset_params.offset_out) {
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) != 0) {
                    m_tet_attribute[t_id].label = 1;
                    // propagate to faces, edges, verts
                    auto v_ids = oriented_tet_vids(t_id);
                    for (const size_t& v_id : v_ids) {
                        m_vertex_extra[v_id].label = 1;
                    }
                    for (int i = 0; i < 6; i++) {
                        m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                    }
                    for (int i = 0; i < 4; i++) {
                        m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                    }
                }
            }
        }
    } else { // not single body mode. must evaluate expression

        // label tets
        auto tets = get_tets();
        for (const Tuple& t : tets) {
            size_t t_id = t.tid(*this);
            if (expr->eval(m_tet_attribute[t_id].tag)) {
                m_tet_attribute[t_id].label = 1;
                // propagate to faces, edges, verts
                auto v_ids = oriented_tet_vids(t_id);
                for (const size_t& v_id : v_ids) {
                    m_vertex_extra[v_id].label = 1;
                }
                for (int i = 0; i < 6; i++) {
                    m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                }
                for (int i = 0; i < 4; i++) {
                    m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                }
            }
        }

        // label faces
        auto faces = get_faces();
        for (const Tuple& f : faces) {
            size_t f_id = f.fid(*this);
            if (m_face_extra[f_id].label == 1) {
                continue;
            }

            // collect incident tags
            CellTag adj_tags = m_tet_attribute[f.tid(*this)].tag;
            auto other = f.switch_tetrahedron(*this);
            if (other) {
                for (const int64_t& tag : m_tet_attribute[other.value().tid(*this)].tag) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_face_extra[f_id].label = 1;
                // propagate to children simplices
                m_edge_attribute[f.eid(*this)].label = 1;
                m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
                m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                m_vertex_extra[f.vid(*this)].label = 1;
                m_vertex_extra[f.switch_vertex(*this).vid(*this)].label = 1;
                m_vertex_extra[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        // label edges
        auto edges = get_edges();
        for (const Tuple& e : edges) {
            size_t e_id = e.eid(*this);
            if (m_edge_attribute[e_id].label == 1) {
                continue;
            }

            // collect adjacent tags
            CellTag adj_tags;
            auto adj_tets = get_incident_tids_for_edge(e);
            for (const size_t& inc_t_id : adj_tets) {
                for (const int64_t& tag : m_tet_attribute[inc_t_id].tag) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_edge_attribute[e_id].label = 1;
                m_vertex_extra[e.vid(*this)].label = 1;
                m_vertex_extra[e.switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        // label vertices
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            if (m_vertex_extra[v_id].label == 1) {
                continue;
            }

            // collect one ring tags
            CellTag adj_tags;
            auto one_ring_t_ids = get_one_ring_tids_for_vertex(v_id);
            for (const size_t& t_id : one_ring_t_ids) {
                for (const int64_t& tag : m_tet_attribute[t_id].tag) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_vertex_extra[v_id].label = 1;
            }
        }
    }

    mark_input_complex_vertices();
}

void TopoOffsetTetMesh::mark_input_complex_vertices()
{
    // The earliest point at which the input complex is known: label_input_complex() has just
    // evaluated the selection expression, whereas init_surfaces_and_boundaries() runs out of
    // init_from_image() and can only see region boundaries -- which is why m_is_on_region is not
    // this.
    //
    // The label is the input complex at every dimension, so this covers a solid complex, a sheet,
    // a wire and an isolated point alike; it must key on the vertex label rather than on incident
    // faces, since the sub-manifold cases have no input face to read a flag off. Interior
    // vertices of a solid complex are included, and the offset surface never reaches them.
    //
    // From here the operations maintain it: a split midpoint ANDs its endpoints, a collapse ORs
    // onto the survivor, and collapse_before_vertex() refuses the merge that would put it on the
    // same vertex as m_is_on_offset.
    size_t n = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const bool on_input = m_vertex_extra[vid].label == 1;
        m_vertex_extra[vid].m_is_on_input_complex = on_input;
        n += on_input ? 1 : 0;
    }
    logger().info("\tInput-complex vertices: {}", n);
}


bool TopoOffsetTetMesh::empty_input_complex()
{
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (m_vertex_extra[v_id].label == 1) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTetMesh::init_input_complex_bvh()
{
    // used a few times, just collect once
    auto tets = get_tets();
    auto faces = get_faces();
    auto edges = get_edges();
    auto verts = get_vertices();

    // to check if a face is in the closure of input complex tets
    std::map<simplex::Face, bool> face_in_closure;
    for (const Tuple& f : faces) {
        size_t v0 = f.vid(*this);
        size_t v1 = f.switch_vertex(*this).vid(*this);
        size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
        face_in_closure[simplex::Face(v0, v1, v2)] = false;
    }

    // to check if an edge is in the closure of input complex tets and faces
    std::map<simplex::Edge, bool> edge_in_closure;
    for (const Tuple& e : edges) {
        size_t v0 = e.vid(*this);
        size_t v1 = e.switch_vertex(*this).vid(*this);
        edge_in_closure[simplex::Edge(v0, v1)] = false;
    }

    // to check if a vertex is in the closure of input complex tets, faces, and edges
    std::map<size_t, bool> vertex_in_closure;
    for (const Tuple& v : verts) {
        vertex_in_closure[v.vid(*this)] = false;
    }

    // collect tets in input complex
    std::vector<simplex::Tet> complex_tets;
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 1) {
            auto vs = oriented_tet_vids(t_id);
            complex_tets.emplace_back(vs[0], vs[1], vs[2], vs[3]);
            face_in_closure[simplex::Face(vs[0], vs[1], vs[2])] = true;
            face_in_closure[simplex::Face(vs[1], vs[2], vs[3])] = true;
            face_in_closure[simplex::Face(vs[2], vs[3], vs[0])] = true;
            face_in_closure[simplex::Face(vs[3], vs[0], vs[1])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[1])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[2])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[3])] = true;
            edge_in_closure[simplex::Edge(vs[1], vs[2])] = true;
            edge_in_closure[simplex::Edge(vs[1], vs[3])] = true;
            edge_in_closure[simplex::Edge(vs[2], vs[3])] = true;
            vertex_in_closure[vs[0]] = true;
            vertex_in_closure[vs[1]] = true;
            vertex_in_closure[vs[2]] = true;
            vertex_in_closure[vs[3]] = true;
        }
    }

    // collect isolated faces in input complex
    std::vector<simplex::Face> complex_faces;
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        size_t v0 = f.vid(*this);
        size_t v1 = f.switch_vertex(*this).vid(*this);
        size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
        simplex::Face f_simp(v0, v1, v2);
        if (!face_in_closure[f_simp] && m_face_extra[f_id].label == 1) {
            complex_faces.emplace_back(v0, v1, v2);
            face_in_closure[f_simp] = true;
            edge_in_closure[simplex::Edge(v0, v1)] = true;
            edge_in_closure[simplex::Edge(v1, v2)] = true;
            edge_in_closure[simplex::Edge(v2, v0)] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
            vertex_in_closure[v2] = true;
        }
    }

    // collect isolated edges in input complex
    std::vector<simplex::Edge> complex_edges;
    for (const Tuple& e : edges) {
        size_t e_id = e.eid(*this);
        size_t v0 = e.vid(*this);
        size_t v1 = e.switch_vertex(*this).vid(*this);
        simplex::Edge e_simp(v0, v1);
        if (!edge_in_closure[e_simp] && m_edge_attribute[e_id].label == 1) {
            complex_edges.emplace_back(v0, v1);
            edge_in_closure[e_simp] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
        }
    }

    // collect isolated vertices
    std::vector<size_t> complex_verts;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (!vertex_in_closure[v_id] && m_vertex_extra[v_id].label == 1) {
            complex_verts.push_back(v_id);
            vertex_in_closure[v_id] = true;
        }
    }

    // extract vertices included in simplicial complex
    std::vector<Vector3d> V_vec;
    std::map<size_t, size_t> v_id_map;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (vertex_in_closure[v_id]) {
            v_id_map[v_id] = V_vec.size();
            V_vec.push_back(m_vertex_attribute[v_id].m_posf);
        }
    }
    MatrixXd V(V_vec.size(), 3);
    for (int i = 0; i < V_vec.size(); i++) {
        V.row(i) = V_vec[i];
    }

    MatrixXi T(complex_tets.size(), 4); // tets
    int index = 0;
    for (const simplex::Tet& t_simp : complex_tets) {
        auto vs = t_simp.vertices();
        // NOTE: does vertex id order matter here?
        T.row(index) << v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]], v_id_map[vs[3]];
        index++;
    }

    MatrixXi F(complex_faces.size(), 3); // faces
    index = 0;
    for (const simplex::Face& f_simp : complex_faces) {
        auto vs = f_simp.vertices();
        // NOTE: does vertex id order matter here?
        F.row(index) << v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]];
        index++;
    }

    MatrixXi E(complex_edges.size(), 2); // edges
    index = 0;
    for (const simplex::Edge& e_simp : complex_edges) {
        auto vs = e_simp.vertices();
        E.row(index) << v_id_map[vs[0]], v_id_map[vs[1]];
        index++;
    }

    MatrixXi P(complex_verts.size(), 1); // isolated vertices
    index = 0;
    for (const size_t& v_id : complex_verts) {
        P(index, 0) = v_id_map[v_id];
        index++;
    }

    // set BVH
    m_input_complex_bvh.clear(); // in case resetting now
    m_input_complex_bvh.init(V, T, F, E, P);

    // The smooth offset potential, from the same extraction and in the same call, so the two
    // descriptions of the input can never diverge.
    //
    // Phi's 3D primitives are triangles, segments and points; there is no volume primitive, so a
    // solid input region enters as its boundary -- the faces of the complex's tets with exactly
    // one incident complex tet. Outside the region, the only place an offset exists, distance to
    // the region and distance to its boundary are the same number. (Inside they differ and Phi
    // has a second, mirrored level set, unreachable because the band grows outward and the
    // runaway guard catches a vertex that crossed the complex.) 2D does the same one dimension
    // down.
    std::map<simplex::Face, int> boundary_count;
    for (const simplex::Tet& t_simp : complex_tets) {
        for (const simplex::Face& f : t_simp.faces()) {
            ++boundary_count[f];
        }
    }

    std::vector<Vector3i> phi_tris;
    for (const auto& [f_simp, count] : boundary_count) {
        if (count != 1) continue; // interior to the complex: carries no boundary geometry
        const auto vs = f_simp.vertices();
        phi_tris.emplace_back(v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]]);
    }
    for (int i = 0; i < F.rows(); ++i) { // isolated triangles of the complex
        phi_tris.emplace_back(F(i, 0), F(i, 1), F(i, 2));
    }

    // The edge list must be complete. ipc derives faces_to_edges from it and throws if a triangle
    // edge is missing, and the OGC feasible-region test for a vertex reads that vertex's edge
    // neighbours, so an incomplete list silently widens every Voronoi region and puts a spurious
    // spherical cap wherever a neighbour went unlisted.
    std::set<std::pair<int, int>> phi_edge_set;
    const auto add_edge = [&](const int a, const int b) {
        phi_edge_set.emplace(std::min(a, b), std::max(a, b));
    };
    for (const Vector3i& t : phi_tris) {
        add_edge(t[0], t[1]);
        add_edge(t[1], t[2]);
        add_edge(t[2], t[0]);
    }
    for (int i = 0; i < E.rows(); ++i) { // isolated edges (wires) of the complex
        add_edge(E(i, 0), E(i, 1));
    }

    MatrixXi F_phi(phi_tris.size(), 3);
    for (size_t i = 0; i < phi_tris.size(); ++i) {
        F_phi.row(i) = phi_tris[i].transpose();
    }
    MatrixXi E_phi(phi_edge_set.size(), 2);
    {
        int i = 0;
        for (const auto& [a, b] : phi_edge_set) {
            E_phi(i, 0) = a;
            E_phi(i, 1) = b;
            ++i;
        }
    }

    std::vector<int> P_phi;
    for (int i = 0; i < P.rows(); ++i) {
        P_phi.push_back(P(i, 0));
    }

    // Kept, not built. The extraction must not diverge from the BVH's, so it is done here and
    // once; the potential itself needs target_distance and offset_dhat_factor, which a caller
    // that only wants the distance field has no reason to have set.
    m_phi_V = V;
    m_phi_E = E_phi;
    m_phi_F = F_phi;
    m_phi_P = P_phi;

    // The input complex needs no containment envelopes of its own: every simplex of it lies on
    // tag-region boundaries (see label_input_complex()), so the per-tag envelopes built in
    // init_surfaces_and_boundaries() -- from the same input partition, before construction
    // touches it -- already hold all of it, junctions included.
}


std::shared_ptr<SampleEnvelope> TopoOffsetTetMesh::envelope_for_mask(uint64_t mask) const
{
    if (mask == 0) return nullptr;
    if ((mask & (mask - 1)) == 0) {
        // Single bit: the member envelope itself -- a real SampleEnvelope, safe on every path
        // including the pull. Linear scan; the tag count is tiny.
        for (const auto& [tag, env] : m_tag_envelopes) {
            const auto it = m_tag_bit.find(tag);
            if (it != m_tag_bit.end() && (mask >> it->second) == 1) return env;
        }
        return nullptr; // a bit whose tag never got an envelope (no boundary faces at init)
    }
    // Several bits: the memoized intersection. Lazy and mutex-guarded because containment
    // queries run concurrently under kPartition; creation is rare (a handful of junction
    // masks per model), so the lock is uncontended in steady state.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        const auto it = m_isect_cache.find(mask);
        if (it != m_isect_cache.end()) return it->second;
    }
    std::vector<std::shared_ptr<SampleEnvelope>> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env);
        }
    }
    std::shared_ptr<SampleEnvelope> isect;
    if (members.empty()) {
        isect = nullptr; // every bit dangled; nothing to contain in
    } else if (members.size() == 1) {
        isect = members.front(); // the other bits dangled; degrade to the one real tube
    } else {
        isect = std::make_shared<IntersectionEnvelope>(std::move(members));
    }
    std::lock_guard<std::mutex> lock(m_isect_mutex);
    m_isect_cache.emplace(mask, isect);
    return isect;
}


void TopoOffsetTetMesh::init_offset_potential()
{
    if (m_phi_V.rows() == 0) {
        log_and_throw_error("init_offset_potential() called before init_input_complex_bvh()");
    }
    // Which field defines the offset; see OffsetPotential.hpp and the offset_field parameter.
    // Both are built from the same extraction -- m_phi_V/E/F/P, from init_input_complex_bvh() --
    // so whichever is chosen measures the same geometry the diagnostics do.
    if (m_offset_params.offset_field == "euclidean") {
        // A query engine, not a tolerance: nearest_point_feature() supplies the foot point and
        // the feature kind the exact derivatives case on, and only the exact kind answers it. eps
        // is never read, since no containment test runs against this object.
        //
        // Triangles where there are any, segments otherwise: the exact envelope is one kind or
        // the other and answers nearest_point_feature() for whichever it was built as.
        std::vector<Eigen::Vector3d> verts(size_t(m_phi_V.rows()));
        for (int i = 0; i < m_phi_V.rows(); ++i) {
            verts[size_t(i)] = m_phi_V.row(i).head<3>();
        }
        m_input_complex_envelope = std::make_shared<SampleEnvelope>();
        m_input_complex_envelope->use_exact = true;
        if (m_phi_F.rows() > 0) {
            std::vector<Eigen::Vector3i> tris(size_t(m_phi_F.rows()));
            for (int i = 0; i < m_phi_F.rows(); ++i) {
                tris[size_t(i)] = Eigen::Vector3i(m_phi_F(i, 0), m_phi_F(i, 1), m_phi_F(i, 2));
            }
            m_input_complex_envelope->init(verts, tris, m_offset_params.target_distance);
            logger().info(
                "\tOffset field: EUCLIDEAN (exact distance), level d = {}, {} triangles",
                m_offset_params.target_distance,
                tris.size());
        } else {
            std::vector<Eigen::Vector2i> segs;
            segs.reserve(size_t(m_phi_E.rows()) + m_phi_P.size());
            for (int i = 0; i < m_phi_E.rows(); ++i) {
                segs.emplace_back(m_phi_E(i, 0), m_phi_E(i, 1));
            }
            // Isolated input points as the degenerate segment (i, i); nearest_feature() demotes
            // a hit on one to the vertex case.
            for (const int i : m_phi_P) {
                segs.emplace_back(i, i);
            }
            m_input_complex_envelope->init(verts, segs, m_offset_params.target_distance);
            logger().info(
                "\tOffset field: EUCLIDEAN (exact distance), level d = {}, {} segments (no "
                "triangles in the input complex)",
                m_offset_params.target_distance,
                segs.size());
        }
        m_offset_potential = std::make_shared<EuclideanOffsetPotential3D>(
            m_input_complex_envelope,
            m_offset_params.target_distance);
        return;
    }

    // dhat is sized to the offset it has to hold, not to target_distance alone. Construction
    // places the offset at the input tetrahedralization's own cell boundaries, so how far out it
    // lands is an absolute property of the input mesh rather than a multiple of delta, and a
    // fixed factor x delta fails when delta is small relative to the background tets.
    //
    // The floor keeps the configured factor authoritative whenever construction was good. dhat is
    // not a neutral scaling -- the level c depends on it -- so a purely data-driven dhat would
    // give the same geometry a different offset depending only on how the input was meshed. With
    // the floor, well-constructed inputs all agree and the measurement rescues only the rest.
    const double delta = m_offset_params.target_distance;
    const double reach = max_band_vertex_distance();
    const double dhat = std::max(m_offset_params.offset_dhat_factor * delta, 2. * reach);
    const double effective_factor = dhat / delta;
    if (reach > 0.) {
        logger().info(
            "\tdhat sized from the constructed offset: furthest offset vertex {:.6g} = {:.4g}x "
            "delta, so dhat = max({}x delta, 2x that) = {:.6g} = {:.4g}x delta",
            reach,
            reach / delta,
            m_offset_params.offset_dhat_factor,
            dhat,
            effective_factor);
    }
    m_offset_potential = std::make_shared<SmoothOffsetPotential3D>(
        m_phi_V,
        m_phi_E,
        m_phi_F,
        m_phi_P,
        delta,
        effective_factor);
}


size_t TopoOffsetTetMesh::flood_fill()
{
    size_t current_id = 0;
    auto verts = get_vertices();
    std::map<size_t, bool> visited_verts;

    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (m_vertex_extra[v_id].label == 0) continue; // vertex not in complex
        if (visited_verts.find(v_id) != visited_verts.end()) continue; // vertex already visited

        visited_verts[v_id] = true;
        m_vertex_extra[v_id].component_id = current_id;
        std::queue<size_t> bfs_queue;

        // initial propagation
        auto onering_verts = connected_components_helper(v_id);
        for (const size_t& other_v_id : onering_verts) {
            if (visited_verts.find(other_v_id) == visited_verts.end()) { // other vertex not visited
                bfs_queue.push(other_v_id);
            }
        }

        while (!bfs_queue.empty()) {
            size_t curr_vid = bfs_queue.front();
            bfs_queue.pop();
            if (visited_verts.find(curr_vid) != visited_verts.end()) continue; // already visited
            visited_verts[curr_vid] = true;
            m_vertex_extra[curr_vid].component_id = current_id;

            // propagate
            auto onering_verts_tmp = connected_components_helper(curr_vid);
            for (const size_t& other_v_id : onering_verts_tmp) {
                if (visited_verts.find(other_v_id) == visited_verts.end()) {
                    bfs_queue.push(other_v_id);
                }
            }
        }
        current_id++;
    }

    return current_id;
}

bool TopoOffsetTetMesh::is_order_2_edge(const Tuple& e) const
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    return is_order_2_edge({{v1, v2}});
}

bool TopoOffsetTetMesh::is_order_2_edge(const std::array<size_t, 2>& e) const
{
    return get_order_of_edge(e) == 2;
}

bool TopoOffsetTetMesh::vertex_is_on_surface(const size_t vid) const
{
    // The domain wall must count here, or it drifts. get_surface_faces_for_vertex() returns
    // nothing for a vertex this answers false for, and the smoother's containment check walks
    // exactly that collection -- so a wall vertex left out is tested against an empty set of
    // faces and every move passes.
    //
    // A different predicate from m_vertex_attribute[].m_is_on_surface, which is set correctly for
    // wall vertices either way; this override is what the substructure walk consults.
    return vertex_is_on_region(vid) || m_vertex_extra.at(vid).m_is_on_offset;
}

bool TopoOffsetTetMesh::face_is_on_surface(const size_t fid) const
{
    return m_face_attribute.at(fid).m_is_surface_fs;
}

size_t TopoOffsetTetMesh::get_order_of_vertex(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_order;
}

void TopoOffsetTetMesh::init_vertex_order()
{
    std::array<size_t, 4> count{{0, 0, 0, 0}};

    for (const Tuple& t : get_vertices()) {
        const size_t vid = t.vid(*this);
        const size_t order = compute_vertex_order(vid);
        m_vertex_attribute[vid].m_order = order;
        count[order]++;
    }

    logger().info("Vertex order count (0,1,2,3): {}", count);
}

void TopoOffsetTetMesh::compute_vertex_partition()
{
    if (NUM_THREADS == 0) {
        return;
    }

    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) { return m_vertex_attribute[i].m_posf; },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); ++i) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

std::vector<std::array<size_t, 3>> TopoOffsetTetMesh::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond) const
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto tid = fid / 4, lid = fid % 4;
            auto verts = get_face_vertices(f);
            res.emplace_back( //
                std::array<size_t, 3>{
                    {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }
    return res;
}


void TopoOffsetTetMesh::execute_offset(const std::filesystem::path& output_file)
{
    // make embedding simplicial (split components per Alg 1)
    logger().info("Creating simplicial embedding...");
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) { // internally prints to console
        simplicial_embedding();
        bool dummy = is_simplicially_embedded(); // internally prints to console
    }
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // initialize offset
    logger().info("Initializing offset...");
    // The inserted vertex is the plain edge midpoint -- target_distance does not enter the
    // placement at all. Carrying the surface out to target_distance is the optimization phase's
    // job; see the note above set_offset_tet_tags().
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    marching_tets();
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // No growth pass: the band is exactly the one layer of background tets marching_tets()
    // labelled from the frontier one-rings, so the offset boundary sits on the input
    // tetrahedralization's own cell boundaries and nothing has widened it toward
    // target_distance. Closing that gap is the optimization phase's job.

    // simplicially embed again, if needed
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) {
        simplicial_embedding();
        bool dummy = is_simplicially_embedded();
    }
    // Must stay outside the branch above and unconditional: consolidating renumbers, which
    // changes the order later passes enumerate operations in, which changes the run. Inside the
    // `if`, a mesh already simplicially embedded would consolidate only when debug output is on.
    // Same reason as the one before the optimization loop.
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // Do not re-add a distance-field marching pass that sphere-traces each band-boundary edge
    // onto d(x) = target_distance at insertion time; measured worse -- see git history of this
    // file. Placing the offset boundary is the optimization phase's job, which is why
    // optimize_offset() is unconditional: construction leaves the band boundary on
    // background-cell boundaries, accurate only to the local cell size until the optimization
    // moves it.
    set_offset_tet_tags();
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    assert(ambient_assert());
}

double
TopoOffsetTetMesh::mean_ratio_metric(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2)
{
    const Vector3d a = p1 - p0;
    const Vector3d b = p2 - p1;
    const Vector3d c = p0 - p2;

    const double sq_length_sum = a.squaredNorm() + b.squaredNorm() + c.squaredNorm();
    if (sq_length_sum < 1e-20) return 0.; // degenerate triangle, treat as maximally bad
    const double area = a.cross(b).norm();

    constexpr double prefactor = 2. * 1.7320508075688772; // 2*sqrt(3)
    return prefactor * area / sq_length_sum;
}

void TopoOffsetTetMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A vertex on both surfaces is unsatisfiable: it sits at distance 0 from the input complex,
    // where Phi diverges, and is required to sit on the level set at target_distance at the same
    // time. No placement satisfies both, so it is a hard error rather than something to step
    // around -- the measurements do step around it (Phase B refuses it, the metric books it as
    // pinned), which is right for them and would otherwise leave a construction defect silent.
    //
    // A vertex on the bounding box is deliberately not checked: it is constrained but not
    // contradictory, sliding on the box and still able to reach the level set where the box
    // permits.
    //
    // Checked after every phase, not only at construction: collapse_after_vertex() ORs the two
    // flags onto the survivor, so a collapse merging an offset vertex into an input one creates
    // this state out of two individually fine vertices.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input_complex) {
            continue;
        }
        // The geometry decides, not the flags. The flag pair is over-broad -- the split
        // propagates it onto new vertices and the collapse ORs it onto survivors -- so a
        // vertex can carry both while sitting a full target_distance from the complex,
        // which is exactly where the offset wants it. Being unsatisfiable is a geometric
        // fact: smoothing_position_is_allowed() holds an input-complex vertex within
        // envelope_size of the complex and the offset asks it to reach target_distance,
        // and those contradict only when the vertex really is on the complex.
        if (m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf)) >
            m_offset_params.envelope_size) {
            continue;
        }
        both.push_back(vid);
    }
    if (both.empty()) {
        return;
    }

    const size_t n_show = std::min<size_t>(both.size(), 8);
    std::string detail;
    for (size_t i = 0; i < n_show; ++i) {
        const size_t vid = both[i];
        const double d = m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf));
        detail += fmt::format("{}{} (dist to input {:.6g})", i ? ", " : "", vid, d);
    }
    log_and_throw_error(
        "[{}] {} vertices are on BOTH the input complex and the offset surface. Such a vertex is "
        "at distance 0 from the input and is asked to be at target_distance {} from it at the "
        "same time, so the optimization cannot place it and the offset surface through it cannot "
        "converge. This is a construction defect, not an optimization failure. Offending "
        "vertices: {}{}",
        when,
        both.size(),
        m_offset_params.target_distance,
        detail,
        both.size() > n_show ? ", ..." : "");
}

void TopoOffsetTetMesh::rebuild_offset_envelope()
{
    std::vector<Eigen::Vector3d> V(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        V[i] = m_vertex_attribute[i].m_posf;
    }
    std::vector<Eigen::Vector3i> F;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        F.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
    }
    if (F.empty()) {
        // Nothing to hold. Not an error: a run whose offset region never formed has other
        // problems, and they are reported where they happen.
        m_offset_envelope = nullptr;
        logger().warn("\t[phase A] no offset-surface faces; the offset envelope is empty");
        return;
    }

    // One width, every round: eps = offset_envelope_rel x the Phi tolerance. Phase A is pinned
    // to the same tube whether the surface is far from the level set or nearly on it. Do not
    // re-add a trust region that scales eps from the previous Phase B's max residual; measured
    // worse -- see git history of this file. A wide early envelope buys rearrangement freedom the
    // loop does not need and pays for it in re-placement work every round.
    //
    // Cannot be 0: refinement moves the surface by about the local chord error when a split
    // lands, and an envelope tighter than that refuses Phase A's operations outright.
    const double tol = offset_residual_tolerance();
    const double eps = std::max(m_offset_params.offset_envelope_rel * tol, 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>();
    m_offset_envelope->use_exact = true;
    m_offset_envelope->init(V, F, eps);
    logger().info(
        "\t[phase A] offset envelope rebuilt: {} faces, eps {:.6g} ({:.4}x tolerance)",
        F.size(),
        eps,
        eps / tol);
}

TopoOffsetTetMesh::GradientSplit TopoOffsetTetMesh::gradient_split(
    const bool include_face_samples) const
{
    // The convergence criterion: |grad (Phi(x) - c)^2| over the offset surface -- at every band
    // vertex, and at interior samples of every band face.
    //
    // Must be the gradient of the same objective Phase B's sweeps minimize for these vertices --
    // the offset term and nothing else. Any drift between the two measures a different fixed
    // point than the one the sweeps converge to.
    //
    // Weight 1, deliberately: this is an absolute bound in length units, so a tuning knob such as
    // w_envelope would scale the bar. (The Phase B stop test may use one, being a ratio against
    // its own value at phase entry, where any positive constant cancels.)
    const std::vector<bool> on_band = band_vertex_mask();
    // Non-const because polysolve's Problem::gradient is non-const; a local's constness is
    // independent of this method's, so no cast is needed.
    OffsetEnergy3D offset_energy(m_offset_potential, 1.0);

    GradientSplit s;
    double sum_reachable = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        const auto& ve = m_vertex_extra[vid];
        if (!ve.m_is_on_offset) continue;

        // Skipped, not measured: a vertex the smoother declines to place this pass has a
        // gradient that is not part of the fixed point. Counted so a run cannot report
        // convergence over a band it never fully measured.
        if (!m_vertex_attribute[vid].m_is_rounded) {
            ++s.n_skipped_unrounded;
            continue;
        }
        const auto locs = get_one_ring_tets_for_vertex(tuple_from_vertex(vid));
        if (locs.empty()) continue;
        bool inverted = false;
        for (const Tuple& loc : locs) {
            if (is_inverted_f(loc)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            ++s.n_skipped_inverted;
            continue;
        }

        Eigen::VectorXd g(3);
        const Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        offset_energy.gradient(x, g);
        const double gn = g.norm();

        // Pinned vertices are reported, not gated -- the one place the gradient criterion
        // deliberately parts company with residual_split(), which counts a pinned vertex toward
        // its driving max. A residual is a statement about the surface, so a pinned vertex off
        // the level set is a real error and belongs in the bound; a gradient is a statement about
        // the iteration, and Phase B never moves an input-complex vertex, so folding it in would
        // make convergence unreachable by construction. Surfaced through max_pinned instead,
        // where the remedy is construction rather than more rounds.
        if (ve.m_is_on_input_complex) {
            s.max_pinned = std::max(s.max_pinned, gn);
            ++s.n_pinned;
            continue;
        }

        if (gn > s.max_reachable) {
            s.max_reachable = gn;
            s.worst_vid = vid;
        }
        s.max_at_vertex = std::max(s.max_at_vertex, gn);
        sum_reachable += gn;
        ++s.n_reachable;
    }

    // ... and across the band's faces, on the same lattice the residual is sampled on.
    //
    // A vertex criterion is not a surface criterion: every vertex can sit exactly on the level
    // set while the triangles between them chord across it, and a vertex-only gradient reads that
    // as converged. E = (Phi(x) - c)^2 is a field, so evaluating it inside a face is the same
    // computation as at a corner, not an approximation of one.
    //
    // The split is kept and reported because the remedies differ: an at-vertex max is the surface
    // in the wrong place and wants smoothing, an in-face max is a surface too coarse to be in the
    // right place and wants refinement. Both are errors in the returned offset, so both gate.
    if (include_face_samples) {
        double sum_faces = 0.;
        for (const Tuple& f : get_faces()) {
            if (!face_is_offset_surface_live(f)) continue;
            // A face with a pinned corner is pinned: no placement of the vertices the optimizer
            // owns can carry its interior to the level set. Booked where a pinned vertex is
            // booked, for the same reason -- see the carve-out above.
            bool pinned = false;
            for (const size_t vid : get_face_vids(f)) {
                if (m_vertex_extra[vid].m_is_on_input_complex) {
                    pinned = true;
                    break;
                }
            }
            for_each_offset_face_sample(f, [&](const Vector3d& q) {
                Eigen::VectorXd g(3);
                offset_energy.gradient(Eigen::VectorXd(q), g);
                const double gn = g.norm();
                if (pinned) {
                    s.max_pinned = std::max(s.max_pinned, gn);
                    return;
                }
                s.max_reachable = std::max(s.max_reachable, gn);
                s.max_in_face = std::max(s.max_in_face, gn);
                sum_faces += gn;
                ++s.n_face_samples;
            });
        }
        sum_reachable += sum_faces;
        s.n_reachable += s.n_face_samples;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

double TopoOffsetTetMesh::phase_b_band_gradient_linf()
{
    // One measurement, one definition: the Phase B stop test and the convergence test must read
    // the same function, or a phase stops on one number while the run is judged by another and
    // sits at a fixed point of neither.
    //
    // Vertices only -- the same function restricted to the variables this phase owns. Phase B
    // moves vertices and performs no topological operation, so the in-face term is constant under
    // everything it can do: a chording triangle still chords after any placement of its corners.
    // Folding it in would pin the stop test at a value the sweeps cannot lower, burning
    // ab_smooth_max_passes every round. The in-face term is Phase A's to fix, through refinement.
    return gradient_split(/*include_face_samples=*/false).max_at_vertex;
}

size_t TopoOffsetTetMesh::phase_b_smooth()
{
    // Smoothing only, to a fixed point. No topology: Phase B's single job is to move the offset
    // surface onto the level set, on whatever mesh Phase A left. Running to convergence rather
    // than for a fixed count is what makes the sizing refinement afterwards meaningful -- a face
    // still over tolerance once nothing moves is under-resolved, not badly placed.
    std::vector<Vector3d> before(vert_capacity());

    // The criterion is the gradient, relative to its value at phase entry. It is zero exactly at
    // the Gauss-Seidel fixed point, so unlike a displacement test it cannot read converged when
    // moves are blocked -- a refused move has zero displacement and full gradient. Entry-relative
    // makes it scale-free across rounds whose Phase A left very different amounts of work.
    const double g_entry = phase_b_band_gradient_linf();
    // Two bars, whichever comes first. The entry-relative one asks whether this phase finished
    // the work it was handed; the absolute one is the run's own convergence bar, without which a
    // phase handed a large entry gradient keeps sweeping long after the run would have been
    // declared converged, since 1% of a large number is still over the bar.
    const double g_abs = offset_gradient_tolerance();
    logger().info(
        "\t[phase B] placement gradient at entry {:.6g}; the run's convergence bar is {:.6g}",
        g_entry,
        g_abs);
    if (g_entry <= 0.) {
        return 0; // already at the fixed point; nothing to smooth
    }

    // Negative ab_smooth_max_passes means uncapped: run until the gradient criterion fires. The
    // only other exit is the no-progress guard below -- a gradient that has stopped falling is a
    // blocked configuration, and looping on it forever helps nobody.
    const size_t cap = m_offset_params.ab_smooth_max_passes < 0
                           ? std::numeric_limits<size_t>::max()
                           : size_t(std::max(1, m_offset_params.ab_smooth_max_passes));
    double g_prev = g_entry;
    int no_progress = 0;
    int constrained_prev = std::numeric_limits<int>::max();
    size_t pass = 0;
    double disp = 0.;
    for (; pass < cap; ++pass) {
        for (size_t i = 0; i < vert_capacity(); ++i) {
            before[i] = m_vertex_attribute[i].m_posf;
        }

        // Two ordered sub-sweeps: first place every offset-surface vertex on the level set, then
        // let every free interior vertex relax against the mesh that placement just distorted.
        // Ordered rather than interleaved so the background always relaxes against the offset's
        // final position for this pass.
        m_phase_b_constrained = 0;
        m_phase_b_sub = PhaseBSub::Offset;
        smooth_all_vertices(1);
        // Captured between the sweeps: only the offset sub-sweep can constrain a placement, and
        // smooth_all_vertices() resets the shared reject counters, so a before/after delta on
        // `accepted` would wrap.
        const int constrained = m_phase_b_constrained.load();
        const size_t placed = m_smooth_rejects.accepted.load();

        m_phase_b_sub = PhaseBSub::Background;
        smooth_all_vertices(1);
        const size_t relaxed = m_smooth_rejects.accepted.load();

        disp = 0.;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            disp = std::max(disp, (m_vertex_attribute[vid].m_posf - before[vid]).norm());
        }
        // Relative to the target edge length, so the test means the same thing at any scale.
        const double tol = m_offset_params.ab_smooth_tol * std::max(m_params.l, 1e-16);

        // The residual per pass, not just once at the end of the phase. The sizing refinement
        // afterwards rests on "Phase B ran to a fixed point, so a face still over tolerance is
        // under-resolved rather than badly placed", which holds only if this loop exits on the
        // gradient criterion below rather than the pass cap. This line tells the two apart: a
        // residual still falling at the cap means the cap is the binding constraint, a residual
        // flat while the gradient is converged means refinement is the right answer.
        //
        // max_at_vertex vs max_in_face says which remedy -- the surface in the wrong place (wants
        // smoothing) or too coarse to be in the right place (wants refinement). See DistanceSplit.
        const DistanceSplit rp = residual_split();
        const double rtol = offset_residual_tolerance();
        logger().info(
            "\t[phase B] pass {}: max vertex displacement {:.6g} (tol {:.6g}) | residual {:.4}x "
            "(at-vertex {:.4}x, in-face {:.4}x) | reachable {} pinned {}",
            pass + 1,
            disp,
            tol,
            rp.max_reachable / rtol,
            rp.max_at_vertex / rtol,
            rp.max_in_face / rtol,
            rp.n_reachable,
            rp.n_pinned);
        const double g = phase_b_band_gradient_linf();
        logger().info(
            "\t[phase B] pass {}: placement gradient {:.6g} ({:.4g}x entry, {:.4g}x the "
            "convergence bar {:.6g})",
            pass + 1,
            g,
            g_entry > 0. ? g / g_entry : 0.,
            g / g_abs,
            g_abs);
        logger().info(
            "\t[phase B] pass {}: placed {} constrained {} | background relaxed {}",
            pass + 1,
            placed,
            constrained,
            relaxed);
        // One frame per smoothing pass, on the same debug_{N} series and counter as the Phase A
        // passes, so a viewer globbing *debug_*.vtu gets both phases in the order they ran.
        // Opt-in through DEBUG_output_per_pass because the series is large. Safe to call here:
        // write_vtu() is observational and does not consolidate, so the frames cannot change the
        // run they are showing.
        if (m_params.debug_output) {
            write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }
        // The natural exit: every offset vertex reached its unconstrained minimum inside its own
        // one-ring, so nothing had to be backtracked -- the fixed point this scheme is defined to
        // seek. Checked before the gradient bar because it is the stronger statement.
        if (constrained == 0) {
            logger().info(
                "\t[phase B] no placement was constrained by its one-ring this pass; the offset "
                "surface is at its achievable fixed point");
            ++pass;
            break;
        }
        // The run's own convergence bar: once the band is under it there is nothing another
        // round could do with a better-placed surface.
        if (g <= g_abs) {
            ++pass;
            break;
        }
        // Both measures must stall. The gradient can sit flat while the constrained count is
        // still falling -- vertices are still being freed from their one-rings even though the
        // worst placement error has not moved -- so stopping on the gradient alone cuts that
        // short. A non-finite gradient is never progress: written as `g < g_prev` alone, an
        // alternating inf -> finite sequence reads as a decrease every other pass, resets this
        // counter, and with an uncapped Phase B never terminates.
        const bool g_better = std::isfinite(g) && g < g_prev;
        const bool c_better = constrained < constrained_prev;
        no_progress = (g_better || c_better) ? 0 : no_progress + 1;
        g_prev = g;
        constrained_prev = std::min(constrained_prev, constrained);
        if (no_progress >= 10) {
            // The plateau is the achievable fixed point: the gradient falls sharply from entry
            // and then sits flat a little above the bar, which is the fixed point rather than a
            // configuration fighting the inversion guard. Nothing more is coming from smoothing,
            // so stop and let the sizing refinement answer the residual.
            logger().warn(
                "\t[phase B] neither the placement gradient nor the constrained count has "
                "improved for {} passes (gradient {:.6g}, {:.4g}x entry, {} still constrained); "
                "treating the plateau as the phase's fixed point and stopping",
                no_progress,
                g,
                g_entry > 0. ? g / g_entry : 0.,
                constrained);
            ++pass;
            break;
        }
    }
    return pass;
}

size_t TopoOffsetTetMesh::update_band_sizing_from_tolerance()
{
    // See the header for the rule. Three passes so each quantity is computed once: Phi's
    // gradient is the expensive part and the face samples dominate it.
    const double gtol = offset_gradient_tolerance();
    const std::vector<bool> on_band = band_vertex_mask();
    OffsetEnergy3D energy(m_offset_potential, 1.0);
    const auto grad_at = [&](const Vector3d& p) {
        Eigen::VectorXd g(3);
        energy.gradient(Eigen::VectorXd(p), g);
        return g.norm();
    };

    // 1. Every band vertex against the criterion. Non-band vertices are left `true` so they
    //    never veto a neighbour's halving -- the rule is about the SURFACE one-ring.
    std::vector<char> in_tol(vert_capacity(), 1), is_band(vert_capacity(), 0);
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid] || !m_vertex_extra[vid].m_is_on_offset) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue; // not placeable, not this rule's
        is_band[vid] = 1;
        in_tol[vid] = grad_at(m_vertex_attribute[vid].m_posf) <= gtol ? 1 : 0;
    }

    // 2. One sweep of the band's faces gives both the surface one-ring and, per vertex,
    //    whether any incident face carries an out-of-tolerance interior sample.
    std::vector<char> ring_in_tol(vert_capacity(), 1), has_bad_face(vert_capacity(), 0);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        bool face_bad = false;
        for_each_offset_face_sample(f, [&](const Vector3d& q) {
            if (!face_bad && grad_at(q) > gtol) face_bad = true;
        });
        for (int i = 0; i < 3; ++i) {
            if (face_bad) has_bad_face[vs[i]] = 1;
            for (int j = 0; j < 3; ++j) {
                if (i != j && !in_tol[vs[j]]) ring_in_tol[vs[i]] = 0;
            }
        }
    }

    // 3. Apply. The floor is the band's, as in refine_sizing_around_worst().
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / m_params.l);
    std::vector<size_t> changed;
    size_t n_halved = 0, n_misplaced = 0, n_ring_blocked = 0, n_at_floor = 0;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!is_band[vid]) continue;
        // Misplaced, not under-resolved: leave it to Phase B. Counted so the two reasons a
        // vertex goes unrefined stay distinguishable in the log -- they are opposite diagnoses.
        if (!in_tol[vid]) {
            ++n_misplaced;
            continue;
        }
        if (!has_bad_face[vid]) continue; // nothing wrong here at all
        if (!ring_in_tol[vid]) {
            ++n_ring_blocked;
            continue;
        }
        double& s = m_vertex_attribute[vid].m_sizing_scalar;
        const double ns = std::max(s_floor, s * 0.5);
        if (ns < s) {
            s = ns;
            ++n_halved;
            changed.push_back(vid);
        } else {
            ++n_at_floor; // wanted refining and cannot -- the floor is binding here
        }
    }
    if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
    const size_t n_band = size_t(std::count(is_band.begin(), is_band.end(), char(1)));
    logger().info(
        "\t[phase B] band sizing: {} of {} halved | not refined: {} misplaced (vertex out of "
        "tolerance), {} ring-blocked (a neighbour is), {} at the floor, {} nothing wrong | "
        "floor {:.6g}",
        n_halved,
        n_band,
        n_misplaced,
        n_ring_blocked,
        n_at_floor,
        n_band - n_halved - n_misplaced - n_ring_blocked - n_at_floor,
        s_floor);
    return changed.size();
}


void TopoOffsetTetMesh::optimize_offset_alternating()
{
    const int rounds = std::max(1, m_offset_params.max_rounds);
    const int a_iters = std::max(1, m_offset_params.max_iterations);

    // Before anything runs, so a construction defect is reported as one rather than surfacing
    // later as a residual that will not converge.
    check_no_vertex_on_both_surfaces("construction");

    // Where the mesh as constructed stands, before the loop touches it. A diagnostic: it is
    // the baseline every later round's residual is read against.
    const double construction_residual = residual_split().max_reachable;
    logger().info(
        "\tconstruction residual: {:.6g} ({:.4}x tolerance)",
        construction_residual,
        construction_residual / offset_residual_tolerance());
    const GradientSplit construction_grad = gradient_split();
    logger().info(
        "\tconstruction placement gradient: {:.6g} ({:.4}x tolerance) | {} reachable, {} pinned, "
        "{} skipped",
        construction_grad.max_reachable,
        construction_grad.max_reachable / offset_gradient_tolerance(),
        construction_grad.n_reachable,
        construction_grad.n_pinned,
        construction_grad.n_skipped_unrounded + construction_grad.n_skipped_inverted);

    // Each run logs its own switch settings, so a log is self-describing.
    logger().info("[experiment switches] w_amips {:g} (Phase A only)", m_params.w_amips);

    // Per-round operation counts. iter_cnt_* are cumulative since optimize_offset() reset them,
    // so a round's contribution is the delta against the previous round's totals.
    int prev_split = iter_cnt_split.load();
    int prev_collapse = iter_cnt_collapse.load();
    int prev_swap = iter_cnt_swap.load();
    int prev_born = iter_cnt_split_born.load();
    int prev_recol = iter_cnt_recollapsed.load();
    int prev_recol_same = iter_cnt_recollapsed_same_pass.load();

    for (int round = 0; round < rounds; ++round) {
        // ---- PHASE A: TetWild, with the offset held inside its envelope ----
        logger().info("======== A/B round {} / {}: phase A ========", round + 1, rounds);
        m_phase = OptPhase::A;
        rebuild_offset_envelope();
        mesh_improvement(a_iters);

        // Ask the loop, in the loop's own units; never recompute the bar by hand.
        // optimization_quality_stats() reports AMIPS against optimization_stop_metric() =
        // stop_energy, while cell_quality is AMIPS^3, so a check written as
        // cell_quality/stop_energy > 1 fails a Phase A that has in fact converged.
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        if (m_params.debug_output) {
            // Named by phase, not by the debug counter: these two driver writes are the
            // per-phase timeline -- the state each phase hands to the next -- and the viewer
            // globs `*phase_*.vtu` to show that series apart from the per-pass debug_{N} frames.
            write_optimization_debug_output(fmt::format("phase_{}A", round + 1));
        }

        // Phase A has to converge: it is TetWild on a mesh TetWild can improve, with the offset
        // surface pinned to a tolerance-wide tube. Element quality still above stop_energy when
        // the loop gives up is something iterating further will not fix, and continuing into
        // Phase B would optimize the offset on a mesh that cannot carry it.
        if (amips > bar) {
            // Which element, before saying only how bad. See log_worst_tet().
            log_worst_tet("phase A gave up");
            log_and_throw_error(
                "Phase A did not converge within {} iterations: max element quality {:.6} against "
                "stop_energy {}. The offset envelope may be too tight (offset_envelope_rel "
                "{}), or the mesh has elements no operation can fix.",
                a_iters,
                amips,
                bar,
                m_offset_params.offset_envelope_rel);
        }
        logger().info("\t[phase A] converged: max element quality {:.4} (stop {:.4})", amips, bar);
        // Phase A collapses can merge an offset vertex into an input one, and
        // collapse_after_vertex() ORs both flags onto the survivor.
        check_no_vertex_on_both_surfaces(fmt::format("round {} phase A", round + 1).c_str());

        // ---- PHASE B: smoothing only, and the stuck check ----
        logger().info("======== A/B round {} / {}: phase B ========", round + 1, rounds);
        m_phase = OptPhase::B;
        // Released, so smoothing can actually move the surface. Phase A rebuilds it next round
        // around wherever this leaves it.
        m_offset_envelope = nullptr;

        // Reclaim the slot pool, or the next Phase A cannot split anything at all.
        //
        // The pool is preallocated at preallocation_factor x the live count at the last
        // consolidate, and slots consumed by operations are returned only by a consolidate, not
        // by the collapses that removed the elements. mesh_improvement() consolidates each
        // iteration, but after its stop test, so a Phase A that meets stop_energy on its first
        // iteration exits without ever consolidating and every later round inherits the pool as
        // it was left. That is the steady state, not a corner case: a Phase A whose split pass
        // inflates the mesh and whose collapse pass returns it spends the slots and frees none,
        // until every split is dropped and the A/B loop repeats identically while reporting
        // progress.
        //
        // Here rather than at the top of Phase A, because refine_sizing_around_worst() queues
        // m_force_split_edges by vertex id and Phase A consumes them. Consolidating renumbers, so
        // a consolidate between the queue and its consumer would feed the split pass identifiers
        // for other vertices.
        const size_t vcap_before = vert_capacity(), tcap_before = tet_capacity();
        consolidate_mesh();
        logger().info(
            "\t[phase B] consolidated: slot capacity #V {} -> {}, #T {} -> {}",
            vcap_before,
            vert_capacity(),
            tcap_before,
            tet_capacity());

        const size_t passes = phase_b_smooth();

        const DistanceSplit r = residual_split();
        report_outside_support("A/B round", r);
        // The loop's convergence test is the gradient. The Phi residual and the Euclidean
        // distance error are measured and logged every round because they answer questions this
        // one cannot -- the residual carries the chord term that ranks the sizing field, the
        // Euclidean error says how far the smoothed offset ended up from the exact one -- but
        // neither gates the run. See offset_gradient_tolerance().
        const GradientSplit g = gradient_split();
        const double phi = g.max_reachable / offset_gradient_tolerance();
        logger().info(
            "\t[phase B] {} smoothing passes, max placement gradient {:.6g} = {:.4}x tolerance "
            "(at-vertex {:.4}x, in-face {:.4}x) | phi residual {:.4}x its own bar | {} reachable, "
            "{} pinned (max {:.6g}), {} skipped ({} unrounded, {} inverted ring)",
            passes,
            g.max_reachable,
            phi,
            g.max_at_vertex / offset_gradient_tolerance(),
            g.max_in_face / offset_gradient_tolerance(),
            r.max_reachable / offset_residual_tolerance(),
            g.n_reachable,
            g.n_pinned,
            g.max_pinned,
            g.n_skipped_unrounded + g.n_skipped_inverted,
            g.n_skipped_unrounded,
            g.n_skipped_inverted);

        // Before the convergence check below, so the round that converges is recorded like every
        // other one rather than dropped by the early return.
        {
            const int s = iter_cnt_split.load();
            const int c = iter_cnt_collapse.load();
            const int w = iter_cnt_swap.load();
            op_counts.push_back({{s - prev_split, c - prev_collapse, w - prev_swap}});
            logger().info(
                "\t[A/B round {}] operations: {} splits, {} collapses, {} swaps "
                "(cumulative {} / {} / {})",
                round + 1,
                s - prev_split,
                c - prev_collapse,
                w - prev_swap,
                s,
                c,
                w);
            prev_split = s;
            prev_collapse = c;
            prev_swap = w;

            // Churn, in the same per-round shape. "recollapsed" counts split-born vertices a
            // collapse removed; "same pass" is the subset the very next collapse pass took out.
            const int b = iter_cnt_split_born.load();
            const int rc = iter_cnt_recollapsed.load();
            const int rs = iter_cnt_recollapsed_same_pass.load();
            const int db = b - prev_born, drc = rc - prev_recol, drs = rs - prev_recol_same;
            churn_counts.push_back({{db, drc, drs}});
            logger().info(
                "\t[A/B round {}] split churn: {} of {} split-born vertices recollapsed "
                "({:.1f}%), {} of them in the collapse pass right after their own split "
                "({:.1f}% of born)",
                round + 1,
                drc,
                db,
                db > 0 ? 100.0 * drc / db : 0.0,
                drs,
                db > 0 ? 100.0 * drs / db : 0.0);
            prev_born = b;
            prev_recol = rc;
            prev_recol_same = rs;
        }

        if (m_params.debug_output) {
            // See the phase A twin above: the per-phase series the viewer globs.
            write_optimization_debug_output(fmt::format("phase_{}B", round + 1));
        }

        if (phi <= 1.0) {
            logger().info(
                "A/B converged after {} round(s): amips {:.4}x, phi {:.4}x",
                round + 1,
                amips,
                phi);
            return;
        }

        // Not converged: re-size the band per vertex against the convergence criterion, and let
        // the next Phase A rebuild the mesh at that resolution. This is the only offset-driven
        // refinement; do not add a second rule that refines around any face over tolerance,
        // including one whose own vertex is misplaced rather than under-resolved -- that
        // reintroduces exactly the refinement this rule withholds.
        update_band_sizing_from_tolerance();
    }

    logger().warn(
        "A/B did not converge in {} rounds (max_rounds); the offset residual is still above "
        "tolerance",
        rounds);
}

std::tuple<double, double> TopoOffsetTetMesh::optimization_quality_stats()
{
    // Phase A is TetWild, so its metric is TetWild's: element quality alone, in units of
    // stop_energy, with no Phi term in the stop test, the stall test or the refinement ranking.
    // The offset is not unattended there -- m_offset_envelope holds it -- and mixing Phi back in
    // makes the two criteria fight, since a Phi that cannot improve holds the metric up and keeps
    // the stall detector firing on a mesh whose quality is still descending.
    //
    // Delegated, not reimplemented, and it must report absolute AMIPS as the base does. Returning
    // a normalized metric here (1.0 means converged, as Phase B does) silently breaks the stall
    // refinement: refine_sizing_around_worst derives its filter from this number and compares it
    // against cbrt(cell_quality), which is absolute, so select_worst_cells returns nothing and no
    // sizing is refined. Units are part of "identical to TetWild".
    if (m_phase == OptPhase::A) {
        return wmtk::TetOptimizerMesh::optimization_quality_stats();
    }

    // Outside Phase A the engine's "quality" is the offset's own criterion: (max, avg) Phi
    // residual over the reachable band, normalized so 1.0 is the tolerance. mesh_improvement()
    // stops when the max drops below optimization_stop_metric() = 1.0, i.e. when the whole offset
    // surface -- its vertices and the interiors of its triangles -- is within tolerance of the
    // level set. The residual is defined everywhere, so "in the wrong place" and "too coarse to
    // be in the right place" are one measurement taken at different points; the Euclidean
    // distance stays a logged diagnostic (see compute_distance_deviation()).
    //
    // Only the reachable band counts: a pinned vertex sits on the input complex, where Phi
    // diverges, or on the domain boundary, where growth ran out of room. Leaving those in holds
    // the metric above the bar forever, so the loop never stops and the stall detector fires
    // every iteration, refining around vertices nothing can help.
    const DistanceSplit r = residual_split();

    // The runaway guard, before anything reports a number derived from Phi: a vertex outside the
    // support has a residual that saturates rather than growing, so the numbers below would
    // under-report it, and no smoothing move can bring it back. Checked here because the driver
    // calls this every iteration and residual_split() has already paid for the measurement.
    report_outside_support("Optimization iteration", r);

    // The max of the two criteria, each over its own target, as 2D returns. Phi alone leaves
    // AMIPS out of the stop test, the stall test and the refinement ranking, so a degenerate
    // element can sit in the mesh unseen for a whole run and the offset criterion becomes the
    // only thing holding the surface together.
    const double tol = offset_residual_tolerance();
    double amips = 0.;
    for (const Tuple& t : get_tets()) {
        amips = std::max(amips, cell_quality_rel(t.tid(*this)));
    }
    const double phi = r.max_reachable / tol;
    logger().info("\t[criteria] amips {:.4}x | phi {:.4}x", amips, phi);
    return {std::max(amips, phi), std::max(amips, r.avg_reachable / tol)};
}

size_t TopoOffsetTetMesh::refine_sizing_around_worst(double max_metric)
{
    // One sizing field, two reasons to refine it.
    //
    // In Phase A this is TetWildMesh::refine_sizing_around_worst verbatim -- ranked by element
    // quality, clamped the same way, seeding the same force-split edges. Phase A is TetWild, and
    // that includes how it responds to a stall; the field it writes is the field Phase B reads
    // and writes, so the refinement each phase asks for accumulates rather than competing.
    //
    // The force-split half matters: TetWild records each worst tet's longest edge and
    // split_all_edges splits exactly those, bypassing the length gate and without touching the
    // sizing field, which is how a stuck sliver is broken up rather than merely surrounded by a
    // finer field.
    if (m_phase == OptPhase::A) {
        const int n_rings = std::max(0, m_params.stuck_refine_rings);
        // Clamped above exactly as TetWild does: without it a single degenerate tet sets the
        // filter so high that refinement sees only the degenerate ones and stops fixing the
        // merely-bad tets it exists for.
        const double filter_energy =
            std::min(std::max(max_metric / 100, m_params.stop_energy), 100.);

        // cell_quality is AMIPS^3, so the energy "max energy" refers to is its cube root.
        const auto worst = wmtk::utils::select_worst_cells(
            tet_capacity(),
            [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
            [this](size_t tid) { return std::cbrt(cell_quality(tid)); },
            filter_energy,
            m_params.stuck_refine_num_worst);
        if (worst.empty()) {
            return 0;
        }

        m_force_split_edges.clear();
        if (m_params.stuck_refine_force_split) {
            for (const auto& [unused_score, tid] : worst) {
                m_force_split_edges.insert(
                    wmtk::utils::longest_edge(
                        oriented_tet_vids(tid),
                        [this](size_t vid) -> const Vector3d& {
                            return m_vertex_attribute[vid].m_posf;
                        }));
            }
        }

        std::vector<size_t> seeds;
        seeds.reserve(4 * worst.size());
        for (const auto& [unused_score, tid] : worst) {
            for (const size_t v : oriented_tet_vids(tid)) seeds.push_back(v);
        }
        const auto region = wmtk::utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
            return get_one_ring_vids_for_vertex_adj(v);
        });

        const auto refined = wmtk::utils::apply_sizing_refinement(
            region,
            m_params.stuck_refine_factor,
            m_params.stuck_refine_min_scalar,
            [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
        gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

        logger().info(
            "[stuck-refine A] {} worst tets (max energy {:.4}, filter {:.4}), refined {} of {} "
            "region vertices",
            worst.size(),
            max_metric,
            filter_energy,
            refined.size(),
            region.size());
        // Every stall, so the trajectory is visible: the same tid recurring means one element is
        // pinned, a changing tid means the stall is moving around the mesh.
        log_worst_tet("phase A stall");
        return refined.size();
    }

    // Outside Phase A: the engine's strategy driven by the offset's criterion. mesh_improvement()
    // calls this only when the metric stalls, and it refines around the worst-scoring
    // offset-surface faces -- the same routine with face_criterion_rel() in place of AMIPS
    // energy, as 2D does.
    //
    // Ranking by face rather than by vertex is the half that matters: a vertex score cannot see a
    // surface decimated to a few large triangles cutting across the offset, since every vertex
    // can sit exactly on the level set while the triangles between them do not. The face score
    // samples the triangle's interior, so an under-resolved face is refined first.
    //
    // Stall-driven and worst-first, rather than halving every offset vertex that fails a
    // criterion each iteration: a feature whose error no resolution can lower would otherwise
    // ratchet to the floor unconditionally and gradation would drag that fine sizing into the
    // surrounding volume.
    //
    // The floor is the band's, deliberately, and 3D differs from 2D here on measurement rather
    // than principle: this is min_edge_length_rel x target_distance over l, a construction
    // quantity bounding how fine the band is built, where the Phase A branch above and every
    // stall refinement in TetWild, TriWild and simwild clamp against stuck_refine_min_scalar.
    // Do not switch this to stuck_refine_min_scalar; measured worse -- see git history of this
    // file. It permits a refinement far finer than chord accuracy asks for, which drives the
    // split pass into slot exhaustion for a worse final residual.
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / m_params.l);

    // Worst offset-surface faces above the criterion (a face already inside tolerance is never a
    // reason to refine), capped at stuck_refine_num_worst.
    std::vector<std::pair<double, std::array<size_t, 3>>> scored;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const double score = face_criterion_rel(f);
        if (score <= 1.0) continue;
        scored.emplace_back(score, get_face_vids(f));
    }
    if (scored.empty()) {
        return 0;
    }
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });
    // stuck_refine_num_worst <= 0 means no cap -- every face above the filter -- which is what
    // wmtk::utils::select_worst_cells does with the same parameter and is its shipped default.
    const size_t n_worst = (m_params.stuck_refine_num_worst > 0)
                               ? std::min<size_t>(scored.size(), m_params.stuck_refine_num_worst)
                               : scored.size();

    std::vector<size_t> seeds;
    seeds.reserve(3 * n_worst);
    for (size_t i = 0; i < n_worst; ++i) {
        for (const size_t v : scored[i].second) seeds.push_back(v);
    }
    // Band-only: the region growth here and the gradation below walk offset-surface vertices,
    // not the full one-ring. The reference implementation runs its length-driven hysteresis on
    // the offset surface child mesh and gives the tet embedding a separate, AMIPS-driven per-edge
    // target, so the surface's fine sizing never touches the volume. This port shares one
    // per-vertex scalar between the two, so growing the band's refinement through the one-ring
    // grades it into the surrounding volume and manufactures a halo of demand nothing asked for,
    // bringing split/collapse churn and slot-pool exhaustion with it.
    //
    // The volume keeps its own scalar, which also restores length_rel's meaning for it, and still
    // refines as much as conformity to the band's splits forces. The Phase A branch above is
    // deliberately not confined: that is TetWild's quality-driven stall response over volume
    // elements -- the separate AMIPS-driven target -- not the band's sizing leaking outward.
    const auto region = wmtk::utils::grow_vertex_region(
        seeds,
        std::max(0, m_params.stuck_refine_rings),
        [this](size_t v) { return get_one_ring_vids_for_vertex_adj(v); });

    std::vector<size_t> refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        s_floor,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });

    // No growth: the sizing field is monotone-down in both dimensions. The paper's Sec. 5.3.3
    // Step 1 raises the target length where the surface is flat, but flatness there is a
    // normal-deviation test this component no longer has, and growth is the only direction that
    // can un-resolve a band the criterion has just paid to resolve.
    gradation_smooth_sizing(m_offset_params.sizing_gradation, refined);

    logger().info(
        "[stuck-refine] worst {} of {} offset faces over tolerance (worst {:.4}x), refined {} of "
        "{} region vertices",
        n_worst,
        scored.size(),
        scored.front().first,
        refined.size(),
        region.size());
    return refined.size();
}

void TopoOffsetTetMesh::init_offset_sizing_field()
{
    // Paper Sec. 5.3.3, Step 1: the sizing field "is defined on each edge of the offset mesh and
    // is INITIALIZED WITH THE CURRENT LENGTH OF EACH EDGE."
    //
    // Required, not a detail. The base's m_sizing_scalar defaults to 1, a target length of
    // m_params.l everywhere, and l is a fraction of the bounding box diagonal -- far longer than
    // the offset surface's own edges. Starting there makes every offset edge a collapse candidate
    // on the first pass and the offset is decimated before any metric is computed; a
    // per-operation guard cannot save it, since each individual collapse of a well-resolved
    // surface barely moves the surface and they pass one at a time.
    //
    // Starting from the current lengths says instead: keep the resolution you have, and change it
    // only where update_band_sizing_from_tolerance() finds a reason to. The field is per-vertex,
    // so a vertex takes the mean length of its incident offset-surface edges.
    const double l = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);

    double raw_sum = 0.;
    int n_seeded = 0, n_background = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);

        // Mean length of the offset-surface edges at this vertex. Collected from the offset faces
        // rather than the one-ring edges so a background edge that merely touches the surface
        // does not drag the seed toward the ambient scale.
        double sum_len = 0.;
        int n = 0;
        std::set<size_t> seen;
        for (const Tuple& f : get_offset_surface_faces_for_vertex(v)) {
            for (const size_t nb : get_face_vids(f)) {
                if (nb == vid || !seen.insert(nb).second) continue;
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
        }
        // Every vertex is seeded, not just the offset surface's: a vertex with no incident offset
        // face falls back to its whole one-ring, the same rule applied to whatever scale it sits
        // at. Leaving the background at the base target is what makes the first collapse pass
        // destructive -- the gate is edge length against the target at its endpoints, so a target
        // of l against actual edges a fraction of that marks essentially every interior edge
        // collapsible, and that pass runs before any criterion has been evaluated.
        if (n == 0) {
            for (const size_t nb : get_one_ring_vids_for_vertex(vid)) {
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
            if (n == 0) continue; // isolated vertex; nothing to measure
            ++n_background;
        } else {
            ++n_seeded;
            raw_sum += sum_len / n;
        }

        m_vertex_attribute[vid].m_sizing_scalar =
            std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
    }
    logger().info(
        "\tOffset sizing seed: {} offset-surface vertices (mean incident length {:.6}) + {} "
        "background vertices from their one-rings (base l {:.6}, l_min {:.6} = {} x "
        "target_distance {}, scalar floor {:.6})",
        n_seeded,
        n_seeded > 0 ? raw_sum / n_seeded : 0.,
        n_background,
        l,
        m_offset_params.min_edge_length,
        m_offset_params.min_edge_length_rel,
        m_offset_params.target_distance,
        s_floor);
}

bool TopoOffsetTetMesh::face_is_offset_surface_live(const Tuple& f) const
{
    const size_t ta = f.tid(*this);
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    if (!opp) {
        // Domain boundary. A band cell here means the band was clipped by the bounding box, and
        // that face is offset surface -- its vertices can never reach the target distance, which
        // is precisely the thing that must be measured rather than hidden.
        return cell_is_offset_band(ta);
    }
    const size_t tb = opp->tid(*this);
    const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
    if (a == b) return false; // both in the band, or neither: not the band's surface
    // The band's inner interface, against the input complex it wraps, sits at distance 0 by
    // construction and would drag the reported error to target_distance everywhere.
    return !cell_is_input_complex(a ? tb : ta);
}


void TopoOffsetTetMesh::diag_offset_bands(const char* tag) const
{
    // Offset-surface edges only, against their own target l*s. The global histogram is dominated
    // by background edges, whose target is the configured length_rel scale; this is the surface
    // the optimization is about.
    std::set<std::array<size_t, 2>> edges;
    size_t n_faces = 0;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        ++n_faces;
        const auto vs = get_face_vids(f);
        for (int i = 0; i < 3; ++i) {
            std::array<size_t, 2> e{{vs[i], vs[(i + 1) % 3]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            edges.insert(e);
        }
    }
    size_t below45 = 0, ok = 0, above43 = 0;
    double s_sum = 0.;
    for (const auto& e : edges) {
        const double L = (m_vertex_attribute[e[0]].m_posf - m_vertex_attribute[e[1]].m_posf).norm();
        const double sc = 0.5 * (m_vertex_attribute[e[0]].m_sizing_scalar +
                                 m_vertex_attribute[e[1]].m_sizing_scalar);
        s_sum += sc;
        const double t = m_params.l * sc;
        if (t <= 0.) continue;
        const double r = L / t;
        if (r < 4. / 5.)
            ++below45;
        else if (r <= 4. / 3.)
            ++ok;
        else
            ++above43;
    }
    // Valence of the band's vertices: the split pass's high-valence gate refuses splits whose
    // link vertex exceeds split_high_valence_threshold incident tets, so a band that
    // accumulates valence stops being refinable regardless of what the sizing field asks.
    std::set<size_t> band_verts;
    for (const auto& e : edges) {
        band_verts.insert(e[0]);
        band_verts.insert(e[1]);
    }
    // And over every vertex: the gate tests the LINK of the split edge, so a high-valence
    // vertex anywhere in the band's halo blocks the band edges it links.
    size_t gval_max = 0, gval_over = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t val = get_one_ring_tids_for_vertex(v).size();
        gval_max = std::max(gval_max, val);
        gval_over += (val > size_t(std::max(0, m_params.split_high_valence_threshold)));
    }
    size_t val_max = 0, val_sum = 0, val_over = 0;
    for (const size_t v : band_verts) {
        const size_t val = get_one_ring_tids_for_vertex(tuple_from_vertex(v)).size();
        val_max = std::max(val_max, val);
        val_sum += val;
        val_over += (val > size_t(std::max(0, m_params.split_high_valence_threshold)));
    }
    const size_t n = edges.size();
    logger().info(
        "\t[offset {}] {} faces, {} edges | L/target: <4/5 {} ({:.1f}%), in band {} ({:.1f}%), "
        ">4/3 {} ({:.1f}%) | mean sizing s {:.4} | band valence max {} mean {:.1f}, {} over "
        "threshold | global valence max {} ({} over)",
        tag,
        n_faces,
        n,
        below45,
        n ? 100. * below45 / n : 0.,
        ok,
        n ? 100. * ok / n : 0.,
        above43,
        n ? 100. * above43 / n : 0.,
        n ? s_sum / n : 0.,
        val_max,
        band_verts.empty() ? 0. : double(val_sum) / band_verts.size(),
        val_over,
        gval_max,
        gval_over);
}

double TopoOffsetTetMesh::max_band_vertex_distance() const
{
    // How far the offset surface ended up from the input complex, as a length. Must be exact (BVH
    // nearest point) rather than a straddle-edge upper bound: an input-to-offset edge can be long
    // and nearly tangential, and dhat is not free to inflate -- it selects the level c, so an
    // overestimate changes which surface the run solves for.
    //
    // Returns 0 when there is no offset surface yet -- the band loop finds no faces and never
    // touches the BVH -- which is the signal to fall back to the configured factor.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }
    double worst = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        worst = std::max(worst, (p - m_input_complex_bvh.nearest_point(p)).norm());
    }
    return worst;
}

std::pair<double, double> TopoOffsetTetMesh::compute_distance_deviation() const
{
    // One entry per vertex, not per face-vertex: a vertex shared by several band-outer faces
    // must count once, or the average is weighted by valence rather than by the surface.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }

    double max_dist = 0., sum_dist = 0.;
    size_t n_verts = 0;
    size_t worst = static_cast<size_t>(-1);
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double d = (p - m_input_complex_bvh.nearest_point(p)).norm();
        const double err = std::abs(d - m_offset_params.target_distance);
        if (err > max_dist) {
            max_dist = err;
            worst = vid;
        }
        sum_dist += err;
        ++n_verts;
    }
    m_worst_dist_vid = worst;
    return {max_dist, n_verts > 0 ? sum_dist / n_verts : 0.};
}

double TopoOffsetTetMesh::band_vertex_residual(const size_t vid) const
{
    // How far this vertex is from the level set Phi = c, as a length. The offset's own error, as
    // opposed to compute_distance_deviation()'s Euclidean diagnostic.
    return m_offset_potential->residual_length(m_vertex_attribute[vid].m_posf);
}

TopoOffsetTetMesh::FaceSamples TopoOffsetTetMesh::offset_face_samples(const Tuple& f) const
{
    // The lattice lives in for_each_offset_face_sample() so the gradient criterion measures the
    // same points -- see gradient_split()'s face loop.
    FaceSamples s;
    for_each_offset_face_sample(f, [&](const Vector3d& q) {
        const double r = m_offset_potential->residual_length(q);
        s.max = std::max(s.max, r);
        s.sum += r;
        ++s.n;
    });
    return s;
}

std::vector<bool> TopoOffsetTetMesh::band_vertex_mask() const
{
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }
    return on_band;
}

TopoOffsetTetMesh::DistanceSplit TopoOffsetTetMesh::residual_split() const
{
    // The band's Phi residual. Every offset-surface vertex and every face sample counts toward
    // the driving max, pinned vertices (on the input complex or the domain wall) included: a
    // pinned vertex far from the level set is a real error in the offset the run returns, so
    // hiding it reports convergence for a surface that is not at target distance. The
    // reachable/pinned split is attribution -- when the max comes from a pinned vertex the remedy
    // is construction (domain size, growth room), not more optimization.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double err = m_offset_potential->residual_length(p);
        s.max_reachable = std::max(s.max_reachable, err);
        s.max_at_vertex = std::max(s.max_at_vertex, err);
        sum_reachable += err;
        ++s.n_reachable;
        if (band_vertex_is_reachable(vid)) {
            // The runaway guard's measurement, taken here rather than in its own traversal: this
            // loop already visits exactly the vertices it cares about, and Phi is the expensive
            // part. check_offset_within_support() turns this into the error.
            if (!m_offset_potential->within_support(p)) {
                ++s.n_outside_support;
                const double d = m_input_complex_bvh.dist(VectorXd(p));
                if (d > s.worst_outside_dist) {
                    s.worst_outside_dist = d;
                    s.worst_outside_vid = vid;
                }
            }
        } else {
            // Attribution only: the vertex already counted toward the max and the average
            // above; this records that the count includes n_pinned vertices nothing can move.
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    // ... and the same measurement across the band's faces, which is what stops a surface whose
    // vertices sit on the level set but whose triangles cut across it from reading as converged.
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const FaceSamples fs = offset_face_samples(f);
        if (fs.n == 0) continue; // no samples asked for (offset_residual_samples <= 0)
        s.max_reachable = std::max(s.max_reachable, fs.max);
        s.max_in_face = std::max(s.max_in_face, fs.max);
        sum_reachable += fs.sum;
        s.n_reachable += fs.n;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

void TopoOffsetTetMesh::check_offset_within_support(const char* when) const
{
    report_outside_support(when, residual_split());
}

void TopoOffsetTetMesh::report_outside_support(const char* when, const DistanceSplit& s) const
{
    if (s.n_outside_support == 0) return;

    log_and_throw_error(
        "{}: {} offset-surface vertices have left the smooth offset potential's support "
        "(dhat = {} = offset_dhat_factor x target_distance {}). The worst is vertex {} at "
        "Euclidean distance {} from the input complex, which is {:.2f}x target_distance. Out "
        "there Phi is identically zero WITH a zero gradient: the smoothing term gives those "
        "vertices no direction back, their residual saturates instead of growing, and the "
        "sizing field refines around vertices nothing can move. Raise offset_dhat_factor if "
        "the offset legitimately has to travel that far, or reduce target_distance.",
        when,
        s.n_outside_support,
        m_offset_potential->dhat(),
        m_offset_params.target_distance,
        s.worst_outside_vid,
        s.worst_outside_dist,
        s.worst_outside_dist / std::max(m_offset_params.target_distance, 1e-16));
}

double TopoOffsetTetMesh::cell_quality_rel(const size_t tid) const
{
    // The cube root is not optional: cell_quality stores AMIPS^3 and stop_energy is a bar on
    // AMIPS, so without it this returns AMIPS^3 / stop_energy and the "fails its target" test
    // score > 1 fires at AMIPS > cbrt(stop_energy) rather than at AMIPS > stop_energy -- every
    // element of a converged mesh then reads as failing. Since face_criterion_rel() takes the max
    // of this and the Phi residual ratio, this term would dominate and the Phi signal beside it
    // would never decide anything.
    //
    // Everything else in both dimensions already takes it: TetOptimizerMesh::quality_rel() is
    // cbrt(cell_quality(tid)) / stop_energy, and 2D needs none because it stores AMIPS directly.
    return std::cbrt(cell_quality(tid)) / std::max(m_params.stop_energy, 1e-16);
}

double TopoOffsetTetMesh::amips_rel_at_face(const Tuple& f) const
{
    double q = cell_quality_rel(f.tid(*this));
    if (const auto opp = f.switch_tetrahedron(*this)) {
        q = std::max(q, cell_quality_rel(opp->tid(*this)));
    }
    return q;
}

double TopoOffsetTetMesh::face_criterion_rel(const Tuple& f) const
{
    // The per-face form of optimization_quality_stats()'s max, restricted to what this face
    // carries. >= 1 means the face fails at least one criterion, which is what makes it a
    // candidate for refinement.
    //
    // The AMIPS term is the same one 2D ranks by, and must stay: a degenerate element is
    // invisible to a Phi-only score, so the sizing field never refines around one and the loop
    // never sees it, while 2D treats it as the worst thing in the mesh. The two dimensions would
    // then need opposite acceptance rules for the same operations.
    const double tol = offset_residual_tolerance();
    double score = amips_rel_at_face(f);
    for (const size_t vid : get_face_vids(f)) {
        score = std::max(score, band_vertex_residual(vid) / tol);
    }
    // Across the face as well, so a triangle too coarse to represent the offset is refined. This
    // is the mechanism that keeps the band resolved at all.
    score = std::max(score, offset_face_samples(f).max / tol);
    return score;
}

void TopoOffsetTetMesh::warn_if_offset_reaches_domain_boundary() const
{
    // A band face with no opposite tet lies on the domain boundary: the band ran out of room
    // before reaching target_distance. Counted in vertices as well as faces because the vertices
    // are what is pinned.
    size_t n_faces = 0, n_verts = 0;
    std::vector<bool> counted(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (f.switch_tetrahedron(*this)) continue; // interior face; the band has room here
        if (!cell_is_offset_band(f.tid(*this))) continue;
        ++n_faces;
        for (const size_t v : get_face_vids(f)) {
            if (!counted[v]) {
                counted[v] = true;
                ++n_verts;
            }
        }
    }
    if (n_faces == 0) return;

    logger().warn(
        "Offset band reaches the domain boundary: {} band faces ({} vertices) lie ON the "
        "bounding box. target_distance ({}) exceeds the clearance between the input complex and "
        "the box, so the offset is CLIPPED there and cannot reach the target distance -- those "
        "vertices are on the frozen bounding box and no operation may move them. They ARE "
        "included in max_dist_err / avg_dist_err (see compute_distance_deviation), so expect the "
        "reported error to be dominated by them and to stay flat across iterations. Reduce "
        "target_distance, or pad the background mesh.",
        n_faces,
        n_verts,
        m_offset_params.target_distance);
}

double TopoOffsetTetMesh::wall_offplane_deviation(const size_t vid) const
{
    if (m_offset_params.box_min.size() < 3 || m_offset_params.box_max.size() < 3) return 0.;
    double dev = 0.;
    for (const int f : m_vertex_attribute[vid].on_bbox_faces) {
        const int k = f / 2;
        const double want = (f % 2 == 0) ? m_offset_params.box_min[k] : m_offset_params.box_max[k];
        dev = std::max(dev, std::abs(m_vertex_attribute[vid].m_posf[k] - want));
    }
    return dev;
}

void TopoOffsetTetMesh::log_vertex_movement(const char* when) const
{
    // Mechanism (1) or (2)? See WallMoveStats: added-deviation moves with no tracked face at the
    // vertex are the vacuous-containment path, and an invariant violation with none of them means
    // the flags are stale rather than the check being blind.
    logger().warn(
        "[wall-moves {}] accepted {} (zero-tracked {}) | added off-plane deviation {} (of those "
        "zero-tracked {}) | largest single step {:.6g}",
        when,
        m_wall_moves.accepted.load(),
        m_wall_moves.accepted_zero_tracked.load(),
        m_wall_moves.added_deviation.load(),
        m_wall_moves.added_dev_zero_tracked.load(),
        m_wall_moves.max_single_step.load());

    // Live census, so "attempted" can be read against how many vertices of each class exist.
    std::array<int, size_t(VClass::Count)> live{};
    for (const Tuple& v : get_vertices()) {
        ++live[size_t(vertex_class(v.vid(*this)))];
    }

    for (size_t c = 0; c < size_t(VClass::Count); ++c) {
        const auto& s = m_move_stats[c];
        const int acc = s.accepted.load();
        logger().warn(
            "[move {}] {:<8} live {:5} | attempted {:8} refused-before {:8} accepted {:8} | "
            "displacement sum {:.6g} max {:.6g} mean {:.6g}",
            when,
            vclass_name(VClass(c)),
            live[c],
            s.attempted.load(),
            s.refused_before.load(),
            acc,
            s.sum_disp.load(),
            s.max_disp.load(),
            acc > 0 ? s.sum_disp.load() / acc : 0.);
    }

    // Is the wall still tracked? A face is containment-checked only if it carries
    // m_is_surface_fs, so if the children of a split wall face do not inherit the flag the check
    // has nothing to test and passes for a vertex that has drifted anywhere at all. Counted
    // rather than reasoned about: a wall face count that fails to grow with the mesh while the
    // wall vertex count does means propagation is the gap.
    {
        // A common wall face, not merely three vertices that each touch the box. Near a box edge
        // or corner the three vertices can sit on different walls while the triangle spans the
        // interior, and such a face is correctly untracked; only a face whose three vertices
        // share a wall index lies in that wall and must be contained.
        size_t n_tracked = 0, n_in_wall = 0, n_in_wall_untracked = 0;
        for (const Tuple& f : get_faces()) {
            const auto vs = get_face_vids(f);
            const auto shared = wmtk::set_intersection(
                wmtk::set_intersection(
                    m_vertex_attribute[vs[0]].on_bbox_faces,
                    m_vertex_attribute[vs[1]].on_bbox_faces),
                m_vertex_attribute[vs[2]].on_bbox_faces);
            const bool tracked = m_face_attribute[f.fid(*this)].m_is_surface_fs;
            if (tracked) ++n_tracked;
            if (!shared.empty()) {
                ++n_in_wall;
                if (!tracked) ++n_in_wall_untracked;
            }
        }
        logger().warn(
            "[move {}] tracked faces {} | faces lying IN a wall {}, of those NOT tracked {}{}",
            when,
            n_tracked,
            n_in_wall,
            n_in_wall_untracked,
            n_in_wall_untracked > 0 ? "  <-- UNCONTAINED" : "");
    }

    // The bounding box, checked positionally rather than inferred from the accept counter.
    // on_bbox_faces holds face indices k*2 (the min side of axis k) and k*2+1 (the max side), and
    // a vertex on such a face was placed by exact equality against box_min[k]/box_max[k], so the
    // invariant is exact: coordinate k must still equal that bound, bit for bit. This needs no
    // snapshot and survives consolidation, unlike a remembered set of positions.
    if (m_offset_params.box_min.size() < 3 || m_offset_params.box_max.size() < 3) {
        logger().warn("[move {}] bbox invariant: not checked (box_min/box_max unset)", when);
        return;
    }
    double worst_dev = 0.;
    size_t n_bbox_v = 0, n_moved = 0, worst_vid = size_t(-1);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const auto& faces = m_vertex_attribute[vid].on_bbox_faces;
        if (faces.empty()) continue;
        ++n_bbox_v;
        bool moved = false;
        for (const int f : faces) {
            const int k = f / 2;
            const double want =
                (f % 2 == 0) ? m_offset_params.box_min[k] : m_offset_params.box_max[k];
            const double dev = std::abs(m_vertex_attribute[vid].m_posf[k] - want);
            if (dev > 0.) moved = true;
            if (dev > worst_dev) {
                worst_dev = dev;
                worst_vid = vid;
            }
        }
        if (moved) ++n_moved;
    }
    // Which vertex, and what the containment check sees when it looks at it: no tracked faces
    // means the check has nothing to test, while tracked faces that are all inside mean the
    // envelope is not the binding constraint.
    if (worst_vid != size_t(-1)) {
        size_t n_f = 0, n_out = 0;
        for (const simplex::Face& f : get_surface_faces_for_vertex(worst_vid).faces()) {
            ++n_f;
            const auto& vs = f.vertices();
            if (surface_triangle_is_outside(vs[0], vs[1], vs[2])) ++n_out;
        }
        const auto& bf = m_vertex_attribute[worst_vid].on_bbox_faces;
        logger().warn(
            "[move {}] worst wall vertex {} at ({:.6g}, {:.6g}, {:.6g}) | on_bbox_faces {} | "
            "m_is_on_surface {} on_input {} on_offset {} | tracked faces at it {} of which "
            "OUTSIDE the envelope {}",
            when,
            worst_vid,
            m_vertex_attribute[worst_vid].m_posf[0],
            m_vertex_attribute[worst_vid].m_posf[1],
            m_vertex_attribute[worst_vid].m_posf[2],
            bf.size(),
            m_vertex_attribute[worst_vid].m_is_on_surface,
            m_vertex_extra[worst_vid].m_is_on_region,
            m_vertex_extra[worst_vid].m_is_on_offset,
            n_f,
            n_out);
    }
    // The bar is the envelope, not zero: the wall is contained rather than pinned, so drifting
    // within eps is the design and only exceeding it is a violation.
    logger().warn(
        "[move {}] bbox invariant: {} vertices on the box, {} off their face, worst deviation "
        "{:.6g} (envelope eps {:.6g}){}",
        when,
        n_bbox_v,
        n_moved,
        worst_dev,
        m_envelope_eps,
        worst_dev > m_envelope_eps ? "  <-- VIOLATED: outside the envelope" : "");
}

void TopoOffsetTetMesh::log_quality_spike(size_t tid, double before, double after, const char* kind)
    const
{
    // Genesis events get their own dump lane. The shared budget stops a flood of ordinary spike
    // dumps, but it is spent by early noise, and the genesis dumps are the only ones that name
    // the operation that created an absurd tet. Genesis is rare, so it bypasses the shared budget
    // under its own small cap.
    const bool genesis = std::string_view(kind) == "genesis";
    const int n = genesis ? ++m_spike_genesis : m_spike_count.load();
    if (genesis) {
        if (m_spike_genesis_dumps.load() >= 8) return;
        ++m_spike_genesis_dumps;
    } else {
        if (m_spike_dumps.load() >= m_spike_dump_budget) return;
        ++m_spike_dumps;
    }

    const auto vids = oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int i = 0; i < 4; ++i) p[i] = m_vertex_attribute[vids[i]].m_posf;
    const double vol = (p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.;

    // The scale the volume has to be judged against: a tet of edge L has volume ~L^3/8.5, so
    // vol/L_max^3 near zero is a sliver and a sane ratio with an absurd AMIPS means the blow-up
    // is in the coordinates, not the shape.
    double l_max = 0., l_min = std::numeric_limits<double>::max();
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (const auto& e : E) {
        const double l = (p[e[0]] - p[e[1]]).norm();
        l_max = std::max(l_max, l);
        l_min = std::min(l_min, l);
    }

    int n_unrounded = 0;
    for (int i = 0; i < 4; ++i) n_unrounded += m_vertex_attribute[vids[i]].m_is_rounded ? 0 : 1;

    logger().warn(
        "[spike {} #{}] tid {} | AMIPS {:.6g} -> {:.6g} | volume {:.6e} | l_min {:.6g} l_max "
        "{:.6g} (l_min/l_max {:.3e}, vol/l_max^3 {:.3e}) | {} of 4 vertices UNROUNDED | label {}",
        kind,
        n,
        tid,
        std::isfinite(before) ? std::cbrt(before) : before,
        std::isfinite(after) ? std::cbrt(after) : after,
        vol,
        l_min,
        l_max,
        l_max > 0 ? l_min / l_max : 0.,
        l_max > 0 ? vol / (l_max * l_max * l_max) : 0.,
        n_unrounded,
        m_tet_attribute[tid].label);
    for (int i = 0; i < 4; ++i) {
        const size_t v = vids[i];
        logger().warn(
            "\t  v{} id {} ({:.6g}, {:.6g}, {:.6g}) | rounded {} | on_input {} on_offset {} | "
            "sizing {:.6g}",
            i,
            v,
            p[i][0],
            p[i][1],
            p[i][2],
            m_vertex_attribute[v].m_is_rounded,
            m_vertex_extra[v].m_is_on_region,
            m_vertex_extra[v].m_is_on_offset,
            m_vertex_attribute[v].m_sizing_scalar);
    }
}

void TopoOffsetTetMesh::log_worst_tet(const char* when) const
{
    // Why the offset's own collapse gates refused this round. Counters reset at each Phase A
    // entry.
    {
        const auto& o = m_offset_collapse_refusals;
        logger().warn(
            "[collapse-refusals {}] offset: invariant {} class-region {} class-offset {} "
            "order2 {} sublink {}",
            when,
            o.invariant.load(),
            o.class_region.load(),
            o.class_offset.load(),
            o.order2.load(),
            o.sublink.load());
    }
    // Cumulative spike count, so each stall line says how many tets crossed into absurd since the
    // run began. See log_quality_spike().
    logger().warn(
        "[spikes {}] {} quality writes above AMIPS {:.6g} | {} GENESIS events (ordinary <{:.6g} "
        "ruined by one operation) | record AMIPS {:.6g} | {} dumped",
        when,
        m_spike_count.load(),
        m_spike_threshold,
        m_spike_genesis.load(),
        m_spike_ordinary,
        m_spike_record,
        m_spike_dumps.load());

    log_vertex_movement(when);

    size_t worst = static_cast<size_t>(-1);
    double worst_q = -1.;
    size_t n_over_stop = 0, n_live = 0;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        const double q = std::cbrt(cell_quality(tid));
        ++n_live;
        if (q > m_params.stop_energy) ++n_over_stop;
        if (q > worst_q) {
            worst_q = q;
            worst = tid;
        }
    }
    if (worst == static_cast<size_t>(-1)) return;

    const auto vids = oriented_tet_vids(worst);
    std::array<Vector3d, 4> p;
    for (int i = 0; i < 4; ++i) p[i] = m_vertex_attribute[vids[i]].m_posf;
    const double vol = (p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.;

    // How many of the four carry m_is_on_region. The flag constrains no operation directly; it is
    // reported because it says which vertices are held in the tighter envelope intersection, so a
    // tet with several is constrained but not unfixable.
    int n_on_input = 0;
    for (int i = 0; i < 4; ++i) n_on_input += m_vertex_extra[vids[i]].m_is_on_region ? 1 : 0;

    logger().warn(
        "[worst-tet {}] tid {} | cbrt(quality) {:.6} vs stop_energy {} | {} of {} tets over stop "
        "| volume {:.6e}{} | label {} | {} of 4 vertices on the input complex",
        when,
        worst,
        worst_q,
        m_params.stop_energy,
        n_over_stop,
        n_live,
        vol,
        is_inverted(tuple_from_tet(worst)) ? " INVERTED" : "",
        m_tet_attribute[worst].label,
        n_on_input);

    for (int i = 0; i < 4; ++i) {
        const size_t v = vids[i];
        const auto& va = m_vertex_attribute[v];
        const auto& ve = m_vertex_extra[v];
        logger().warn(
            "\t  v{} id {} ({:.4}, {:.4}, {:.4}) | on_input {} on_offset {} on_bbox {} rounded {} "
            "| order {} sizing {:.4} | valence {}",
            i,
            v,
            p[i][0],
            p[i][1],
            p[i][2],
            ve.m_is_on_region,
            ve.m_is_on_offset,
            va.on_bbox_faces.size(),
            va.m_is_rounded,
            va.m_order,
            va.m_sizing_scalar,
            get_one_ring_tids_for_vertex(tuple_from_vertex(v)).size());
    }

    // The six edges against the three length gates that decide this tet's fate. `split?` is the
    // split pass's candidate test; `collapse-cand?` is the sizing-scaled filter
    // is_weight_up_to_date applies to the collapse candidate list, above which an edge is never
    // offered to collapse at all; `coarsen-gate` is the bounded coarsening pass's unscaled bound.
    // All three must be printed: the unscaled bound alone hides the dead band between them, where
    // an edge is neither splittable nor a collapse candidate.
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (const auto& e : E) {
        const size_t a = vids[e[0]], b = vids[e[1]];
        const double len2 = (p[e[0]] - p[e[1]]).squaredNorm();
        const double sbar =
            0.5 * (m_vertex_attribute[a].m_sizing_scalar + m_vertex_attribute[b].m_sizing_scalar);
        const bool splittable = len2 >= m_params.splitting_l2 * sbar * sbar;
        const bool collapse_cand = len2 <= m_params.collapsing_l2 * sbar * sbar;
        logger().warn(
            "\t  e({},{}) len {:.5} | split? {} (needs >= {:.5}) | collapse-cand? {} (needs <= "
            "{:.5}){} | coarsen-gate? {} (needs <= {:.5})",
            e[0],
            e[1],
            std::sqrt(len2),
            splittable,
            std::sqrt(m_params.splitting_l2) * sbar,
            collapse_cand,
            std::sqrt(m_params.collapsing_l2) * sbar,
            (!splittable && !collapse_cand) ? "  <-- DEAD BAND" : "",
            len2 <= m_params.collapsing_l2,
            std::sqrt(m_params.collapsing_l2));
    }
}

void TopoOffsetTetMesh::log_worst_dist_vertex() const
{
    const size_t vid = m_worst_dist_vid;
    if (vid == static_cast<size_t>(-1)) return;

    const Vector3d p = m_vertex_attribute[vid].m_posf;
    const double d = (p - m_input_complex_bvh.nearest_point(p)).norm();

    // Every face incident to vid, classified. Walked here rather than through
    // get_offset_surface_faces_for_vertex(), which filters to offset faces only -- the point of
    // this line is what ELSE is touching the vertex.
    int n_offset_f = 0, n_input_f = 0, n_bbox_f = 0;
    std::set<size_t> seen_fids;
    for (const size_t tid : get_one_ring_tids_for_vertex(tuple_from_vertex(vid))) {
        const auto tet_vids = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[skip] == vid) continue; // the one face not containing vid
            std::array<size_t, 3> face_vids;
            int k = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) face_vids[k++] = tet_vids[j];
            }
            const auto [face_tuple, unused_tid] = tuple_from_face(face_vids);
            if (!seen_fids.insert(face_tuple.fid(*this)).second) continue;
            n_offset_f += face_is_offset_surface_live(face_tuple);
            n_input_f += face_is_region(face_tuple.fid(*this));
            n_bbox_f += !face_tuple.switch_tetrahedron(*this).has_value();
        }
    }
    const auto& ve = m_vertex_extra[vid];
    // Which face, not just how much. The vertex/face split above says whether the surface is in
    // the wrong place or too coarse to be in the right place; this says where, and at what edge
    // length -- which is what tells refinement from an irreducible feature.
    {
        double worst = 0.;
        std::array<size_t, 3> worst_f{{0, 0, 0}};
        for (const Tuple& f : get_faces()) {
            if (!face_is_offset_surface_live(f)) continue;
            const FaceSamples fs = offset_face_samples(f);
            if (fs.n > 0 && fs.max > worst) {
                worst = fs.max;
                worst_f = get_face_vids(f);
            }
        }
        if (worst > 0.) {
            const Vector3d a = m_vertex_attribute[worst_f[0]].m_posf;
            const Vector3d b = m_vertex_attribute[worst_f[1]].m_posf;
            const Vector3d c = m_vertex_attribute[worst_f[2]].m_posf;
            const auto d_in = [&](const Vector3d& p) {
                return (p - m_input_complex_bvh.nearest_point(p)).norm();
            };
            logger().info(
                "\tworst offset FACE ({}, {}, {}): edge lengths {:.5} {:.5} {:.5} (target_distance "
                "{}), residual inside {:.5} | euclid dist at corners {:.5} {:.5} {:.5}, at "
                "centroid {:.5} | sizing scalars {:.4} {:.4} {:.4} (floor {:.4})",
                worst_f[0],
                worst_f[1],
                worst_f[2],
                (b - a).norm(),
                (c - b).norm(),
                (a - c).norm(),
                m_offset_params.target_distance,
                worst,
                d_in(a),
                d_in(b),
                d_in(c),
                d_in(Vector3d((a + b + c) / 3.)),
                m_vertex_attribute[worst_f[0]].m_sizing_scalar,
                m_vertex_attribute[worst_f[1]].m_sizing_scalar,
                m_vertex_attribute[worst_f[2]].m_sizing_scalar,
                std::max(
                    m_offset_params.min_sizing_scalar,
                    m_offset_params.min_edge_length / std::max(m_params.l, 1e-16)));
            const int k = m_offset_params.offset_residual_samples;
            const int n = k + 2;
            std::string per_sample;
            for (int i = 1; i < n; ++i) {
                for (int j = 1; j < n - i; ++j) {
                    const int lg = n - i - j;
                    if (lg < 1) continue;
                    const Vector3d q = (double(i) * a + double(j) * b + double(lg) * c) / double(n);
                    per_sample += fmt::format(
                        "({},{},{}) d {:.5} r {:.5} phi {:.5} | ",
                        i,
                        j,
                        lg,
                        d_in(q),
                        m_offset_potential->residual_length(q),
                        m_offset_potential->value(q));
                }
            }
            // Per sample, not just the max. A sample whose Phi is far from its neighbours' at
            // essentially the same Euclidean distance is the signature of a second primitive
            // switching on, which refinement cannot fix.
            logger().info(
                "\t  samples on that face (c = {:.5}): {}",
                m_offset_potential->target_level(),
                per_sample);
        }
    }

    logger().info(
        "\tworst-dist vertex {}: pos ({:.6}, {:.6}, {:.6}) dist {:.6} target {:.6} err {:.6}",
        vid,
        p[0],
        p[1],
        p[2],
        d,
        m_offset_params.target_distance,
        std::abs(d - m_offset_params.target_distance));
    logger().info(
        "\t  flags: on_offset {} on_input {} on_bbox {} rounded {} | incident faces: {} offset, "
        "{} input, {} bbox",
        ve.m_is_on_offset,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        n_offset_f,
        n_input_f,
        n_bbox_f);
}

void TopoOffsetTetMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset...");

    // From here on every edge split is an optimization split, run by the shared engine.
    // Marching tets and simplicial embedding are done, and their placement modes -- which
    // require one endpoint inside the offset and one outside -- do not apply to an arbitrary
    // long edge. split_edge_before/after dispatch on this.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    for (const Tuple& t : get_tets()) {
        m_tet_attribute[t.tid(*this)].m_quality = get_quality(t);
    }

    // label all vertices between different tags as fixed
    logger().info("\tLabel offset faces...");
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (m_face_extra[fid].label != 2) {
            continue; // face not on offset, skip
        }
        const auto f_opp = f.switch_tetrahedron(*this);
        if (!f_opp) {
            log_and_throw_error("Offset face {} has no opposite tet.", fid);
        }
        if (m_tet_attribute[f.tid(*this)].label ==
            m_tet_attribute[f_opp.value().tid(*this)].label) {
            continue; // face not between different labels, skip
        }

        // label face and vertices as on offset
        m_face_attribute[fid].m_is_surface_fs = true;
        m_face_attribute[fid].m_surface_class = OFFSET_SURFACE_CLASS;

        auto vs = get_face_vids(f);
        for (const size_t& vid : vs) {
            m_vertex_extra[vid].m_is_on_offset = true;
            // The base's union flag, and the one the shared operations read. It must be set here:
            // TetOptimizerMesh::is_edge_on_surface() short-circuits on it before looking at the
            // face attributes, so without it no offset edge is recognised as carrying tracked
            // geometry and the collapse's surface link condition, preserve_topology, and the
            // split's propagation of the flag are all dead code for the offset.
            m_vertex_attribute[vid].m_is_on_surface = true;
        }
    }

    init_vertex_order();

    // The offset surface as CONSTRUCTED must already be inside the potential's support, or
    // nothing the optimization does can move it. Checked before any operation runs so that a
    // construction defect is reported as one.
    check_offset_within_support("Offset as constructed");

    // Spelled out including the slope^2 factor, so the line reproduces its own number. Without it
    // the criterion reads as rel x target_distance, which is only what it is on a distance field.
    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2| <= {} = front_conv_rel {} x "
        "target_distance "
        "{} x level-set slope^2 {:.6} -- i.e. residual <= {} ({} x target_distance) | diagnostic "
        "Phi residual bar {} (= half the gradient bound, in model units) | level c {:.6}, dhat "
        "{:.6}, {} interior samples per offset face",
        offset_gradient_tolerance(),
        m_offset_params.front_conv_rel,
        m_offset_params.target_distance,
        m_offset_potential->level_set_slope() * m_offset_potential->level_set_slope(),
        0.5 * m_offset_params.front_conv_rel * m_offset_params.target_distance,
        0.5 * m_offset_params.front_conv_rel,
        offset_residual_tolerance(),
        m_offset_potential->target_level(),
        m_offset_potential->dhat(),
        m_offset_params.offset_residual_samples);

    // Seed the sizing field from the offset surface's current edge lengths, before any operation
    // runs. Must come after the offset faces are classified above, since it reads them.
    init_offset_sizing_field();

    /**
     * All label attributes are ignored from here on out. All relevant information is stored in tags
     * and in the "on_surface" and "on_offset" attributes.
     */

    // Unconditional, and it must stay that way: write_vtu() calls consolidate_mesh(), which
    // renumbers the mesh, which changes the order every subsequent pass enumerates operations in,
    // which changes the run. If the debug write below were the only consolidate here, turning
    // DEBUG_output on would silently produce a different numerical result. Consolidating whether
    // or not anything is written keeps DEBUG_output what it claims to be: output only.
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // The shared engine's own loop drives the offset's convergence criterion; the offset plugs
    // into mesh_improvement() the way simwild does, through three virtuals:
    //   - optimization_quality_stats(): (max, avg) over the band, normalized by its tolerance;
    //   - optimization_stop_metric(): 1.0, so the loop stops when the worst is inside tolerance;
    //   - refine_sizing_around_worst(): fired only when the metric stalls -- see the override.
    // The engine consolidates every iteration and re-collects the operation queue, which is also
    // its answer to slot-pool exhaustion: work dropped in one pass is retried in the next, and
    // the [slots] warning reports any pass that came up short.
    //
    // Do not re-add a hand-rolled loop that halves the sizing scalar at every offset vertex
    // failing a criterion each iteration; measured worse -- see git history of this file. A
    // feature whose error no resolution can lower ratchets to the floor regardless of whether
    // refinement is helping, and gradation drags that fine sizing into the surrounding volume.
    iter_cnt_split = 0;
    iter_cnt_split_born = 0;
    iter_cnt_recollapsed = 0;
    iter_cnt_recollapsed_same_pass = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();

    diag_offset_bands("pre");

    // Frame 0 is the mesh as constructed, before the optimization touches it. The shared driver
    // only writes a frame after each operation pass, so without this the timeline starts at the
    // end of the first split with nothing to compare against. Consumes counter 0, so the driver's
    // own frames run from 1 and the viewer's ordering is unchanged.
    if (m_params.debug_output) {
        write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    optimize_offset_alternating();

    // Cumulative over the whole run, not per iteration as before: the engine loop has no
    // per-iteration hook, and the per-pass numbers it logs itself carry the history.
    log_smooth_trace();
    logger().info(
        "splits = {} (offset-edge: {} offered, {} refused by the base's before-gate; "
        "{} reached after_cells with both endpoints on-offset; {} splits produced an "
        "on-offset vertex; {} splits refused in after)  |  "
        "collapses = {} ({} "
        "removed an offset vertex, {} refused by the offset criterion)  |  swaps = {} ({} "
        "refused by the offset criterion)",
        iter_cnt_split.load(),
        iter_cnt_split_offset_before.load(),
        iter_cnt_split_offset_base_reject.load(),
        iter_cnt_split_offset_endpoints.load(),
        iter_cnt_split_offset.load(),
        iter_cnt_split_offset_reject.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_offset_removed.load(),
        iter_cnt_collapse_offset_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_offset_reject.load());
    // No push_back here: op_counts is a per-A/B-round series recorded inside the driver loop, so
    // appending the run totals would make the last entry mean something different from every
    // other one. The run total is the sum of the series.

    // Final metrics and the convergence verdict. One entry for the whole run: the engine loop
    // owns the iterations, so optimization_metrics is a summary rather than a history.
    //
    // One criterion, not two. The paper's termination test pairs a max on the distance error with
    // an average on the normal deviation, an asymmetry forced by geometry -- normal deviation has
    // a floor at every sharp feature that no refinement can lower, and an average cannot be the
    // engine's max-based stop. The Phi residual has no such floor and is defined everywhere on
    // the surface, so "is the surface in the right place" and "is it fine enough to be in the
    // right place" are one measurement taken at vertices and at face interiors, whose max is the
    // engine's stop metric. The Euclidean distance error remains a diagnostic: it says how far
    // the smoothed offset ended up from the exact one, which the design trades away at reentrant
    // features on purpose.
    diag_offset_bands("final");
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    const double tol = offset_residual_tolerance();
    const double gtol = offset_gradient_tolerance();
    logger().info(
        "placement gradient: max {} (avg {}) vs tolerance {} = {} x target_distance | at "
        "vertices {}, inside faces {} ({} face samples) | {} "
        "reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        gtol,
        m_offset_params.front_conv_rel,
        g.max_at_vertex,
        g.max_in_face,
        g.n_face_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    // Both diagnostics, neither a criterion. The Phi residual is the criterion's own quantity in
    // length units -- same vertices, same face lattice -- so it says in model units what the
    // gradient says in gradient units; the Euclidean error says how far the smoothed offset ended
    // up from the exact one.
    logger().info(
        "phi residual (diagnostic, absolute model units): max {} (avg {}) vs bar {} | at "
        "vertices {}, inside faces {} | {} samples, {} pinned vertices || euclid dist err: max {} "
        "| avg {}",
        r.max_reachable,
        r.avg_reachable,
        tol,
        r.max_at_vertex,
        r.max_in_face,
        r.n_reachable,
        r.n_pinned,
        max_dist,
        avg_dist);
    optimization_metrics.push_back(
        {{max_dist,
          avg_dist,
          r.max_reachable,
          r.avg_reachable,
          g.max_reachable,
          g.avg_reachable,
          g.max_at_vertex,
          g.max_in_face}});
    log_worst_dist_vertex();

    m_converged = g.max_reachable <= gtol;
    if (m_converged) {
        logger().info(
            "Converged ([max placement gradient] {} <= {} [front_conv_rel x "
            "target_distance]); worst term {} (at-vertex {}, in-face {})",
            g.max_reachable,
            gtol,
            g.max_in_face > g.max_at_vertex ? "in-face" : "at-vertex",
            g.max_at_vertex,
            g.max_in_face);
        // A band with unmeasured vertices is not a fully measured band, and the criterion is a
        // max: saying so at the moment of the verdict is the only place it cannot be missed.
        if (const size_t skipped = g.n_skipped_unrounded + g.n_skipped_inverted; skipped > 0) {
            logger().warn(
                "Converged with {} band vertices NOT measured ({} unrounded, {} with an inverted "
                "ring). The smoother refuses to place these, so their gradient is not part of the "
                "fixed point -- but the verdict above is a max over the rest.",
                skipped,
                g.n_skipped_unrounded,
                g.n_skipped_inverted);
        }
        if (g.n_pinned > 0 && g.max_pinned > gtol) {
            logger().warn(
                "{} pinned band vertices are over the gradient tolerance (max {} > {}). These are "
                "on the input complex, so Phase B never moves them and no further round can lower "
                "this -- it is a construction defect, not a convergence one.",
                g.n_pinned,
                g.max_pinned,
                gtol);
        }
    }

    if (!m_converged) {
        // Which term failed is the first thing to act on, and naming a vertex when the max came
        // from a face interior points at the wrong remedy: an in-face max is a surface too coarse
        // to represent the level set, which wants refinement and has no vertex to blame.
        // at-vertex is the surface in the wrong place, wants smoothing, and is the only case with
        // a worst_vid.
        if (g.max_in_face > g.max_at_vertex) {
            logger().warn(
                "Optimization did not converge ([max placement gradient] {} > {} "
                "[front_conv_rel x target_distance]); worst term is IN-FACE ({} vs "
                "{} at "
                "vertices) -- the band is too coarse to represent the level set, which wants "
                "refinement rather than smoothing",
                g.max_reachable,
                gtol,
                g.max_in_face,
                g.max_at_vertex);
        } else {
            logger().warn(
                "Optimization did not converge ([max placement gradient] {} > {} "
                "[front_conv_rel x target_distance]); worst term is AT-VERTEX, at "
                "vertex {} "
                "({} vs {} inside faces)",
                g.max_reachable,
                gtol,
                g.worst_vid,
                g.max_at_vertex,
                g.max_in_face);
        }

        // There is deliberately no "blocked by topological preservation" warning here:
        // respect_all_topologies constrains nothing in 3D, so pointing the user at it would send
        // them after a knob that cannot help. (2D still honours it -- see TopoOffsetTriMesh.) A
        // diagnostic for topological blocking has to be written against what actually refuses on
        // topology: the link conditions in split, collapse and swap.
    }

    // Escalate to a hard failure if the caller asked for it, AFTER the warnings above so the log
    // still names which criterion missed before the throw. Guarded on !m_converged alone rather
    // than on the block above: a run that produced no metrics at all has certainly not converged.
    if (!m_converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above for the criterion that failed.",
            optimization_metrics.size(),
            m_offset_params.max_iterations);
    }
}

bool TopoOffsetTetMesh::is_simplicially_embedded() const
{
    int bad_tets = 0;
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        bad_tets += (!tet_is_simp_emb(t));
    }
    if (bad_tets == 0) {
        logger().info("\tBoundary simplicially embedded: TRUE");
        return true;
    } else {
        logger().info("\tBoundary simplicially embedded: FALSE ({} bad tets)", bad_tets);
        return false;
    }
}


bool TopoOffsetTetMesh::tet_is_simp_emb(const Tuple& t) const
{
    size_t t_id = t.tid(*this);
    if (m_tet_attribute[t_id].label != 0) { // entire tet in input/offset
        return true;
    }

    auto vs = oriented_tet_vids(t);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_extra[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }
    if (vs_in.size() <= 1) { // nothing or just one vertex in input
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t glob_eid = tuple_from_edge({{vs_in[0], vs_in[1]}}).eid(*this);
        return (m_edge_attribute[glob_eid].label != 0);
    } else if (vs_in.size() == 3) { // potentially one face in input
        auto [_, glob_fid] = tuple_from_face({{vs_in[0], vs_in[1], vs_in[2]}});
        return (m_face_extra[glob_fid].label != 0);
    } else { // all four verts in complex but tet isn't, can't be simplicially embedded
        return false;
    }
}


void TopoOffsetTetMesh::simplicial_embedding()
{
    // identify necessary tets to split (by vertices)
    std::vector<simplex::Tet> tets_to_split;
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);
        if (m_tet_attribute[tet.tid(*this)].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 4; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 4];
                size_t v3 = vs[(i + 2) % 4];
                auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
                if (m_face_extra[glob_fid].label == 0) {
                    to_split = false;
                    break;
                }
            }

            if (to_split) { // tet not in input but all faces are
                tets_to_split.push_back(simplex::Tet(vs[0], vs[1], vs[2], vs[3]));
            }
        }
    }

    // actually split tets
    for (const simplex::Tet& tet : tets_to_split) {
        const auto& vs = tet.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2], vs[3]);
        std::vector<Tuple> garbage;
        if (!split_tet(t, garbage)) {
            log_and_throw_error("tet split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tTets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_extra[f.fid(*this)].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t glob_eid = tuple_from_edge({{v1, v2}}).eid(*this);
                if (m_edge_attribute[glob_eid].label == 0) {
                    to_split = false;
                }
            }

            if (to_split) {
                faces_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split faces
    for (const simplex::Face& f : faces_to_split) {
        const auto& vs = f.vertices();
        auto [t, _] = tuple_from_face({{vs[0], vs[1], vs[2]}});
        std::vector<Tuple> garbage;
        if (!split_face(t, garbage)) {
            log_and_throw_error("face split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tFaces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 0) { // edge not in input
            size_t v1_id = e.vid(*this);
            size_t v2_id = switch_vertex(e).vid(*this);
            if ((m_vertex_extra[v1_id].label != 0) && (m_vertex_extra[v2_id].label != 0)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        std::vector<Tuple> garbage;
        if (!split_edge(t, garbage)) {
            log_and_throw_error("edge split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}

void TopoOffsetTetMesh::marching_tets()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);

        // if one background and other in input/offset
        if ((m_vertex_extra[v1].label == 0) != (m_vertex_extra[v2].label == 0)) {
            e_to_split.emplace_back(v1, v2);
        }
    }

    // sort edges (split longest first), should give better output mesh quality
    if (m_offset_params.sorted_marching) {
        logger().info("\tSorting edges by length...");
        sort_edges_by_length(e_to_split);
    }

    // actually split edges
    std::vector<Tuple> garbage;
    std::vector<size_t> frontier_verts; // one ring of these verts are in offset
    for (const simplex::Edge& e : e_to_split) {
        // determine which vertex in input/offset
        size_t v_in = e.vertices()[0];
        if (m_vertex_extra[v_in].label == 0) {
            v_in = e.vertices()[1];
        }

        // split edge
        garbage.clear();
        Tuple t = tuple_from_edge(e.vertices());
        if (split_edge(t, garbage)) { // should never fail
            frontier_verts.push_back(v_in);
        } else {
            log_and_throw_error("edge split failed! (marching_tets)");
        }
    }

    // mark all offset tets and children
    for (const size_t& v_id : frontier_verts) {
        auto t_ids = get_one_ring_tids_for_vertex(v_id);
        for (const size_t& t_id : t_ids) {
            if (m_tet_attribute[t_id].label == 0) {
                m_tet_attribute[t_id].label = 2;
                // propagate to children
                for (int i = 0; i < 4; i++) {
                    size_t f_id = tuple_from_face(t_id, i).fid(*this);
                    if (m_face_extra[f_id].label != 1) {
                        m_face_extra[f_id].label = 2;
                    }
                }
                for (int i = 0; i < 6; i++) {
                    size_t e_id = tuple_from_edge(t_id, i).eid(*this);
                    if (m_edge_attribute[e_id].label != 1) {
                        m_edge_attribute[e_id].label = 2;
                    }
                }
                auto vs = oriented_tet_vids(t_id);
                for (const size_t& v_id : vs) {
                    if (m_vertex_extra[v_id].label != 1) {
                        m_vertex_extra[v_id].label = 2;
                    }
                }
            }
        }
    }
}


void TopoOffsetTetMesh::set_offset_tet_tags()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 2) {
            CellTag new_tag;

            // add existing protected tags
            for (const int64_t& existing_tag : m_tet_attribute[t_id].tag) {
                if (std::find(
                        m_offset_params.protected_tags.begin(),
                        m_offset_params.protected_tags.end(),
                        m_tag_id_to_name[existing_tag]) != m_offset_params.protected_tags.end()) {
                    new_tag.insert(existing_tag);
                }
            }

            // add actual offset tags
            if (m_offset_output_tag_ids.size() == 0) {
                if (new_tag.size() == 0) { // no protected tags, write ambient
                    new_tag.insert(0);
                }
            } else {
                for (const int64_t& tag : m_offset_output_tag_ids) {
                    new_tag.insert(tag);
                }
            }

            m_tet_attribute[t_id].tag = new_tag;
        }
    }
}


bool TopoOffsetTetMesh::offset_is_manifold()
{
    // collect tets in closed offset (labelled 1 or 2)
    auto tets = get_tets();
    std::vector<Vector4i> offset_tets;
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        // Region membership from the tags the operations propagate, not the derived label.
        if (cell_in_region(t_id)) {
            auto vs = oriented_tet_vids(t_id);
            offset_tets.emplace_back(vs[0], vs[1], vs[2], vs[3]);
        }
    }

    // construct tet matrix
    MatrixXi T(offset_tets.size(), 4);
    for (int i = 0; i < offset_tets.size(); i++) {
        T.row(i) = offset_tets[i];
    }

    // extract boundary
    MatrixXi F;
    igl::boundary_facets(T, F);

    // remove unreferenced
    VectorXi I;
    VectorXi J;
    igl::remove_unreferenced(F.maxCoeff() + 1, F, I, J);
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < F.cols(); j++) {
            F(i, j) = I(F(i, j));
        }
    }

    // check manifoldness
    bool is_edge_man = igl::is_edge_manifold(F);
    VectorXi B;
    bool is_vert_man = igl::is_vertex_manifold(F, B);
    return (is_edge_man && is_vert_man);
}


bool TopoOffsetTetMesh::invariants(const std::vector<Tuple>& tets)
{
    /**
     * TODO: Check this individually in each operation
     */
    wmtk::utils::predicates::exactinit();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        auto res = wmtk::utils::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);

        if (res != wmtk::utils::predicates::Orientation::NEGATIVE) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTetMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(vertex_size(),
                             -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<paraviewo::CellElement> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset; // vertices to offset
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_extra[i].label == 1) {
            verts_to_offset.push_back(m_vertex_attribute[i].m_posf);
            vid_map[i] = verts_to_offset.size() - 1;
        }
    }
    Eigen::MatrixXd V(verts_to_offset.size(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = verts_to_offset[i];
    }

    // get all offset input edges
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            std::vector<int> curr_e;
            curr_e.push_back(vid_map[e.vid(*this)]);
            curr_e.push_back(vid_map[switch_vertex(e).vid(*this)]);
            paraviewo::CellElement curr_e_elem;
            curr_e_elem.vertices = curr_e;
            curr_e_elem.ctype = paraviewo::CellType::Line;
            cells.push_back(curr_e_elem);
        }
    }

    // get all offset input faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        if (m_face_extra[f.fid(*this)].label == 1) {
            std::vector<int> curr_f;
            curr_f.push_back(vid_map[f.vid(*this)]);
            Tuple f1 = switch_vertex(f);
            curr_f.push_back(vid_map[f1.vid(*this)]);
            Tuple f2 = switch_vertex(switch_edge(f1));
            curr_f.push_back(vid_map[f2.vid(*this)]);
            paraviewo::CellElement curr_f_elem;
            curr_f_elem.vertices = curr_f;
            curr_f_elem.ctype = paraviewo::CellType::Triangle;
            cells.push_back(curr_f_elem);
        }
    }

    // get all offset input tets
    int num_tets = tet_size();
    for (size_t i = 0; i < num_tets; i++) {
        if (m_tet_attribute[i].label == 1) {
            auto vids = oriented_tet_vids(i);
            std::vector<int> curr_t;
            for (const size_t vid : vids) {
                curr_t.push_back(vid_map[vid]);
            }
            paraviewo::CellElement curr_t_elem;
            curr_t_elem.vertices = curr_t;
            curr_t_elem.ctype = paraviewo::CellType::Tetrahedron;
            cells.push_back(curr_t_elem);
        }
    }

    // output
    paraviewo::VTUWriter writer;
    writer.write_mesh(path + ".vtu", V, cells);
}


void TopoOffsetTetMesh::write_phi_grid(const std::string& path, const int n) const
{
    // A plane slice of the offset potential, carrying Phi per grid vertex. A dense volume is
    // unaffordable in 3D, so the slice is the plane through the box centre with the shortest
    // extent removed.
    if (n < 2 || !m_offset_potential) return;

    const Vector3d lo = m_offset_params.box_min, hi = m_offset_params.box_max;
    const Vector3d ext = hi - lo;
    int normal = 0;
    for (int d = 1; d < 3; ++d)
        if (ext[d] < ext[normal]) normal = d;
    const int a0 = (normal + 1) % 3, a1 = (normal + 2) % 3;
    const double at = 0.5 * (lo[normal] + hi[normal]);

    MatrixXd V(n * n, 3);
    MatrixXi F(2 * (n - 1) * (n - 1), 3);
    MatrixXd phi(n * n, 1), residual(n * n, 1), euclid(n * n, 1);
    const double level = m_offset_potential->target_level();

    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const int k = j * n + i;
            Vector3d p;
            p[normal] = at;
            p[a0] = lo[a0] + ext[a0] * i / (n - 1);
            p[a1] = lo[a1] + ext[a1] * j / (n - 1);
            V.row(k) = p.transpose();
            // Phi diverges on the input complex, so it is clamped above to keep the colour map
            // readable. The zero outside the support must never be clamped away: it marks the
            // dead region the placement step cannot escape from.
            phi(k, 0) = std::min(m_offset_potential->value(p), 8. * level);
            residual(k, 0) = std::min(
                m_offset_potential->residual_length(p),
                8. * m_offset_params.target_distance);
            euclid(k, 0) = (p - m_input_complex_bvh.nearest_point(p)).norm();
        }
    }
    int f = 0;
    for (int j = 0; j + 1 < n; ++j) {
        for (int i = 0; i + 1 < n; ++i) {
            const int a = j * n + i, b = a + 1, c = a + n, d = c + 1;
            F.row(f++) << a, b, d;
            F.row(f++) << a, d, c;
        }
    }
    logger().info(
        "Write {}_phi.vtu ({}x{} samples on the {} = {:.6g} plane; the offset is the isosurface "
        "phi = {:.6g}, level-set slope {:.6g})",
        path,
        n,
        n,
        "xyz"[normal],
        at,
        level,
        m_offset_potential->level_set_slope());
    auto writer = std::make_shared<paraviewo::VTUWriter>();
    writer->add_field("phi", phi);
    writer->add_field("phi_residual_length", residual);
    writer->add_field("euclidean_distance", euclid);
    writer->write_mesh(path + "_phi.vtu", V, F, paraviewo::CellType::Triangle);
}

void TopoOffsetTetMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu (tag for offset is included)", path);

    // Writing debug output must not change the mesh, so never consolidate here: consolidation
    // renumbers vertices, and get_partition_id() is keyed on vertex id, so it would change which
    // thread owns which vertex and hence the order operations are applied in.
    //
    // Only the output is compacted, locally. Point arrays stay capacity-sized and slot-indexed,
    // so every vid stays valid and dead slots are unreferenced points; only the cell arrays are
    // packed, since a dead tid would emit a degenerate cell.
    const auto& vs = get_vertices();
    const auto& tets = get_tets();
    const auto faces_in = get_faces_by_condition(
        [](auto& f) { return f.m_is_surface_fs && f.m_surface_class != OFFSET_SURFACE_CLASS; });
    const auto faces_off = get_faces_by_condition(
        [](auto& f) { return f.m_is_surface_fs && f.m_surface_class == OFFSET_SURFACE_CLASS; });
    std::vector<simplex::Edge> edges;
    for (const Tuple& t : get_edges()) {
        simplex::Edge e = simplex_from_edge(t);
        if (is_order_2_edge(e.vertices())) {
            edges.push_back(e);
        }
    }

    MatrixXd V(vert_capacity(), 3);
    MatrixXi T(tets.size(), 4);
    MatrixXi F_in(faces_in.size(), 3);
    MatrixXi F_off(faces_off.size(), 3);
    MatrixXi E(edges.size(), 2);

    V.setZero();
    T.setZero();
    F_in.setZero();
    F_off.setZero();
    E.setZero();

    // last matrix is offset
    std::vector<MatrixXd> tags(m_tags_count + 1, MatrixXd(tets.size(), 1));
    VectorXd labels(vert_capacity());
    labels.setZero();
    VectorXd v_order(vert_capacity());
    v_order.setZero();
    VectorXd v_id(vert_capacity());
    v_id.setZero();
    VectorXd v_sizing(vert_capacity());
    v_sizing.setZero();

    for (size_t k = 0; k < tets.size(); ++k) {
        const size_t t_id = tets[k].tid(*this);

        // set tet tags -- row k, the packed index, not the slot
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](k, 0) = (m_tet_attribute[t_id].tag.count(i) == 1) ? 1 : 0;
        }
        tags[m_tags_count](k, 0) = (m_tet_attribute[t_id].label == 2) ? 1 : 0;
    }

    for (size_t i = 0; i < faces_in.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_in(i, j) = faces_in[i][j];
        }
    }

    for (size_t i = 0; i < faces_off.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_off(i, j) = faces_off[i][j];
        }
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        E(i, 0) = edges[i].vertices()[0];
        E(i, 1) = edges[i].vertices()[1];
    }

    for (const Tuple& v : vs) {
        size_t vid = v.vid(*this);
        labels[vid] = m_vertex_extra[vid].label;
        v_order[vid] = m_vertex_attribute[vid].m_order;
        v_id[vid] = vid;
        v_sizing[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    // set tet verts -- row k as above; the ENTRIES stay vids, matching capacity-sized V
    for (size_t k = 0; k < tets.size(); ++k) {
        const auto& loc_vs = oriented_tet_vertices(tets[k]);
        for (int j = 0; j < 4; j++) {
            T(k, j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    paraviewo::VTUWriter writer;
    for (int64_t i = 0; i < m_tags_count; i++) {
        writer.add_cell_field(m_tag_id_to_name[i], tags[i]);
    }
    writer.add_cell_field("offset_tag", tags[m_tags_count]);
    writer.add_field("labels", labels);
    writer.add_field("order", v_order);
    writer.add_field("vid", v_id);
    writer.add_field("sizing", v_sizing);
    writer.write_mesh(path + ".vtu", V, T, paraviewo::CellType::Tetrahedron);

    // surface
    const std::string surf_out_path = path + "_surf.vtu";
    {
        paraviewo::VTUWriter surf_writer;
        surf_writer.add_field("order", v_order);
        surf_writer.add_field("vid", v_id);
        surf_writer.add_field("sizing", v_sizing);
        logger().info("Write {}", surf_out_path);
        surf_writer.write_mesh(surf_out_path, V, F_in, paraviewo::CellType::Triangle);
    }

    // offset faces
    const std::string off_out_path = path + "_off.vtu";
    {
        paraviewo::VTUWriter off_writer;
        off_writer.add_field("order", v_order);
        off_writer.add_field("vid", v_id);
        off_writer.add_field("sizing", v_sizing);
        logger().info("Write {}", off_out_path);
        off_writer.write_mesh(off_out_path, V, F_off, paraviewo::CellType::Triangle);
    }
    // edges
    const std::string edge_out_path = path + "_edge.vtu";
    {
        paraviewo::VTUWriter edge_writer;
        edge_writer.add_field("order", v_order);
        edge_writer.add_field("vid", v_id);
        edge_writer.add_field("sizing", v_sizing);
        logger().info("Write {}", edge_out_path);
        edge_writer.write_mesh(edge_out_path, V, E, paraviewo::CellType::Line);
    }
}


void TopoOffsetTetMesh::write_msh_groups(const std::string& file)
{
    logger().info("Write {}.msh, {} tags", file, m_tags_count);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& tets = get_tets();

    std::vector<Tuple> tets_with_tag;
    tets_with_tag.reserve(tets.size());

    // add vertices
    const auto& verts = get_vertices();
    msh.add_tet_vertices(verts.size(), [&](size_t k) {
        auto i = verts[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    // The sizing field as node data, so the result file carries what the run asked for and not
    // only what it produced; scripts/visualize_offset.py reads it by this name. Must stay
    // immediately after the vertex block: MshData binds an attribute to entity_blocks.back(),
    // and the per-tag loop below appends an empty vertex block per group.
    msh.add_tet_vertex_attribute<1>("sizing", [&](size_t k) {
        return m_vertex_attribute[verts[k].vid(*this)].m_sizing_scalar;
    });

    auto msh_add_tets = [&]() {
        msh.add_tets(tets_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tet_vids(tets_with_tag[k]);
            std::array<size_t, 4> data;
            for (int j = 0; j < 4; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // add ambient
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].tag.count(0) != 0) {
            tets_with_tag.push_back(t);
        }
    }
    msh_add_tets();
    msh.add_physical_group("ambient");

    // group for each tag
    for (int64_t tag_img = 1; tag_img < m_tags_count; tag_img++) {
        tets_with_tag.clear();
        for (const Tuple& t : tets) {
            size_t t_id = t.tid(*this);
            if (m_tet_attribute[t_id].tag.count(tag_img) != 0) {
                tets_with_tag.push_back(t);
            }
        }

        if (tets_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(3);
        msh_add_tets();

        const std::string group_name = m_tag_id_to_name[tag_img];
        msh.add_physical_group(group_name);
    }

    // if (m_has_envelope) {
    //     msh.add_face_vertices(m_V_envelope.rows(), [this](size_t k) {
    //         return m_V_envelope.row(k);
    //     });
    //     msh.add_faces(m_F_envelope.rows(), [this](size_t k) { return m_F_envelope.row(k); });
    //     msh.add_physical_group("EnvelopeSurface");
    // }

    msh.save(file + ".msh", true);
}


} // namespace wmtk::components::topological_offset
