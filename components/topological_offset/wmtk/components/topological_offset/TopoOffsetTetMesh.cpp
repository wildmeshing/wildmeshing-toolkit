
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

    // One mask bit per input tag, ambient included, in id order. Assigned HERE, once the maps
    // are complete and before init_surfaces_and_boundaries() seeds the vertex masks from the
    // boundary faces. Tags introduced later (the band's offset tag) get no bit -- boundary
    // membership is a property of the INPUT partition, which is also why the masks are
    // propagated rather than ever recomputed from current tet tags.
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
    // THE DOMAIN WALL IS A REGION BOUNDARY. A face with no opposite tetrahedron is the outer
    // boundary of the mesh -- the bounding box -- and it used to be skipped here, on the implicit
    // grounds that a boundary between two tags needs two tags. But it IS one: the boundary
    // between the ambient region and the space outside the domain, and the only reason it had no
    // second tag to compare against is that the outside is not meshed.
    //
    // Skipping it left the box held by a different mechanism entirely -- the on_bbox_faces
    // attribute, which freezes those vertices against smoothing outright and refuses to split
    // any edge lying in the wall. Measured on specific_models/prism, that combination is what
    // manufactured the AMIPS spike: a tet resting on the wall has its longest edge (the wall
    // diagonal, 13.6034) unsplittable and its three wall vertices unmovable, so every legal
    // operation left the worst dimension untouched while halving the volume. The coordinate
    // sequence the splits produced reads -4.77666, -4.77899, -4.78015, -4.78074, -4.78103,
    // -4.78117 against a wall at -4.78132: an exact bisection cascade onto the plane, AMIPS
    // 230 -> 5243 within one pass and 6.9e21 over the run.
    //
    // Held in the region envelope instead, the wall may be refined and its vertices may move
    // within eps of where they started, which is the same contract every other region boundary
    // gets and leaves the degenerate element repairable.
    std::vector<Eigen::Vector3i> tempF;
    std::map<int64_t, std::vector<Eigen::Vector3i>> tag_faces; // per-tag boundary buckets
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        // WHOSE boundary this face is. Interior face: every tag on exactly one side (the
        // symmetric difference -- multi-tag tets exist, and a tag present on both sides has no
        // boundary here). Wall face: every tag of its single tet, the boundary against the
        // unmeshed outside -- which is how ambient's envelope comes to hold the wall.
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
        // THIS SETS THE REGION FLAG, and the name it is stored under is the reason a vertex on
        // both surfaces looked possible.
        //
        // Every face reaching here bounds a region; the ones with a tet on both sides are
        // interior boundaries and the rest are the domain wall, which carries no flag because
        // vertex_is_on_region() reads it off on_bbox_faces. What the flag does NOT mean, despite
        // its name, is "on the input complex": on the double_sphere lens the complex is 302
        // faces and this loop walks 3050, so it lands on the sphere surfaces the offset is grown
        // THROUGH. That is why 134 vertices came out of construction carrying it alongside
        // m_is_on_offset, and why the both-surfaces check grew a geometric escape hatch to
        // tolerate them. The complex itself is m_is_on_input_complex, set from the labels in
        // mark_input_complex_vertices() once label_input_complex() has run.
        if (t_opp) {
            for (const size_t v : {v1, v2, v3}) m_vertex_extra[v].m_is_on_region = true;
        }
        // The base's own flag, which is a DIFFERENT field from the two above: those say which of
        // the offset's two tracked surfaces a vertex belongs to, this says that it belongs to one
        // at all. Every surface-aware path in the shared engine gates on it -- see
        // optimize_offset(), where the same omission had teeth.
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
        // FROM THE PARAMETERS, as the 2D twin already did -- see
        // TopoOffsetTriMesh::init_region_boundary_envelope_from_input(), which is the same line.
        // Without it m_envelope_eps kept its -1 sentinel and the exact envelopes were built with
        // a NEGATIVE half-width, so is_outside() answered true for every triangle, including the
        // ones they had just been constructed from -- freezing every boundary solid.
        //
        // params.init() runs in topological_offset() before init_from_image(), so envelope_size
        // is already resolved from envelope_size_rel x the bbox diagonal by the time we read it.
        // Unit tests construct without params.init(), which is why this whole block stays behind
        // the non-throwing tempF guard rather than adopting a throw on a nonpositive eps.
        m_envelope_eps = m_offset_params.envelope_size;

        // ONE EXACT ENVELOPE PER TAG, from that tag's boundary bucket -- the input partition as
        // it stands BEFORE offset construction rewrites tags, which is what makes E_t the tube
        // around the as-loaded geometry the offset potential also measures against. A boundary
        // face between two regions enters both regions' envelopes; the wall enters its tets'
        // tags' (ambient's, mostly). See the m_tag_envelopes doc in the header for the
        // intersection semantics this feeds.
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

        // The base's pointer survives as the UNION of the members -- inside any tube -- because
        // the one shared-engine site that still reads it directly (the collapse_edge_before
        // point check) asks exactly that question. Everything else dispatches per simplex
        // through envelope_for_mask().
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
    // THE ONLY PLACE THE INPUT COMPLEX IS KNOWN. This runs at the end of label_input_complex(),
    // which has just evaluated the selection expression; init_surfaces_and_boundaries() runs
    // earlier, out of init_from_image(), and can only see region boundaries -- which is exactly
    // why m_is_on_region, set there, is not this.
    //
    // The label is the input complex at every dimension, so this covers a solid complex, a
    // sheet, a wire and an isolated point alike -- the sub-manifold cases have no input FACE to
    // read the flag off, which is why this keys on the vertex label rather than on incident
    // faces. Interior vertices of a solid complex are included; they are inside the region the
    // band wraps and the offset surface never reaches them, so the invariant is untouched.
    //
    // From here the operations maintain it: a split midpoint takes the AND of its endpoints and
    // a collapse ORs onto the survivor, and collapse_before_vertex() refuses the merge that
    // would put it on the same vertex as m_is_on_offset.
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

    // THE SMOOTH OFFSET POTENTIAL, from the same extraction and in the same call, so the two
    // descriptions of the input can never diverge.
    //
    // Phi's 3D primitives are triangles, segments and points; there is no volume primitive. A
    // solid input region therefore enters as its BOUNDARY -- the faces of the complex's tets
    // that have exactly one incident complex tet. Outside the region, which is the only place an
    // offset exists, "distance to the region" and "distance to its boundary" are the same
    // number, so nothing is lost. (Inside the region they differ, and Phi has a second, mirrored
    // level set in there; it is unreachable, because the band is grown outward and the runaway
    // guard would catch a vertex that crossed the complex.) This is exactly what
    // TopoOffsetTriMesh::init_input_complex_bvh does one dimension down.
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

    // The edge list is NOT optional and not a convenience. ipc derives faces_to_edges from it and
    // throws if a triangle edge is missing, and the OGC feasible-region test for a vertex reads
    // that vertex's edge neighbours -- so an incomplete list would silently widen every Voronoi
    // region and put a spurious spherical cap wherever a neighbour went unlisted.
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

    // KEPT, not built. The extraction is what must not diverge from the BVH's, so it is done
    // here and once; the potential itself needs target_distance and offset_dhat_factor, which a
    // caller that only wants the distance field (the unit tests build a TopoOffsetTetMesh from a
    // default-constructed Parameters) has no reason to have set.
    m_phi_V = V;
    m_phi_E = E_phi;
    m_phi_F = F_phi;
    m_phi_P = P_phi;

    // The input complex needs no containment envelopes of its own any more: every simplex of
    // it lies on tag-region boundaries (see label_input_complex()), so the per-tag envelopes
    // built in init_surfaces_and_boundaries() -- from the same input partition, before
    // construction touches it -- already hold all of it, junctions included.
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
    // WHICH FIELD DEFINES THE OFFSET; see OffsetPotential.hpp and the offset_field parameter.
    // Both are built from the SAME extraction -- m_phi_V/E/F/P, which init_input_complex_bvh()
    // produced -- so whichever is chosen measures the same geometry the diagnostics do.
    if (m_offset_params.offset_field == "euclidean") {
        // A QUERY ENGINE, not a tolerance: nearest_point_feature() supplies the foot point and
        // the feature kind the exact derivatives are cased on, and only the exact kind answers
        // it. eps is never read, since no containment test is run against this object.
        //
        // TRIANGLES WHERE THERE ARE ANY, segments otherwise. The exact envelope is one kind or
        // the other and answers nearest_point_feature() for whichever it was built as; a complex
        // with faces wants the face/edge/vertex cases, and a wire or point cloud has only the
        // latter two.
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

    // DHAT IS SIZED TO THE OFFSET IT HAS TO HOLD, not to target_distance alone.
    //
    // Construction places the offset at the input tetrahedralization's own cell boundaries --
    // midpoint marching, no target_distance in it at all -- so how far out it lands is an
    // ABSOLUTE property of the input mesh, not a multiple of delta. A fixed factor x delta
    // therefore fails exactly when delta is small relative to the background tets: measured on
    // two_spheres, the constructed offset reached 5.15x delta at target_distance_rel 0.005 and
    // the factor-2 support rejected 2783 vertices at construction.
    //
    // The floor keeps the configured factor authoritative whenever construction was good. That
    // matters because dhat is NOT a neutral scaling: c = Phi at delta from flat input grows
    // roughly as 1.14 ln(factor) - 0.74, so a purely data-driven dhat would give the same
    // geometry a different offset near edges, corners and gaps depending only on how the input
    // was meshed. With the floor, well-constructed inputs all agree and the measurement only
    // rescues the ones construction placed badly.
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
    // THE DOMAIN WALL COUNTS, and leaving it out is what let the wall drift once it stopped
    // being frozen. get_surface_faces_for_vertex() returns nothing at all for a vertex this
    // answers false for -- it is the first line of that function -- and the containment check
    // in the smoother walks exactly that collection, so a wall vertex was tested against an
    // empty set of faces and every move passed. Measured: all 2409 accepted smoothing moves on
    // a wall vertex had zero tracked faces at them, and the 402 of those that pushed the vertex
    // off its plane took it up to 4.12 in a single step against an envelope half-width of
    // 0.0837.
    //
    // Note this is a DIFFERENT predicate from m_vertex_attribute[].m_is_on_surface, which was
    // set correctly for wall vertices all along -- the worst offender reported
    // "m_is_on_surface true, on_input false, on_offset false", the base attribute and this
    // override disagreeing. The override is what the substructure walk consults.
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


bool TopoOffsetTetMesh::skip_optimization()
{
    static const bool on = [] {
        const char* v = std::getenv("WMTK_OFFSET_SKIP_OPTIMIZE");
        const bool enabled = (v != nullptr) && (std::string(v) == "1");
        if (enabled) {
            logger().warn(
                "WMTK_OFFSET_SKIP_OPTIMIZE=1: optimize_offset() will NOT run. What gets written "
                "is the offset exactly as construction left it -- simplicial embedding plus "
                "midpoint marching_tets(), with no growth pass and no target_distance anywhere "
                "in the placement -- so the offset surface sits on the input "
                "tetrahedralization's own cell boundaries at whatever distance those happen to "
                "be. optimization_metrics stays empty and the report omits that section. "
                "DIAGNOSTIC ONLY; do not read the written distances as an offset.");
        }
        return enabled;
    }();
    return on;
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

    // NO GROWTH PASS. The band is exactly the one layer of background tets marching_tets()
    // labelled from the frontier one-rings, so the offset boundary sits on the input
    // tetrahedralization's own cell boundaries and nothing has widened it toward
    // target_distance. Measured on prism, that leaves the offset surface between 0.24x and 5.54x
    // target_distance from the input complex, against 0.07x-1.93x when a conservative growth
    // pass ran here. Closing that gap is the optimization phase's job.

    // simplicially embed again, if needed
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) {
        simplicial_embedding();
        bool dummy = is_simplicially_embedded();
    }
    // Outside the branch above, and unconditional. write_vtu() consolidates, so leaving this
    // inside the `if` meant that on a mesh already simplicially embedded the ONLY consolidate
    // here was the debug one -- and consolidating renumbers, which changes the order later passes
    // enumerate operations in, which changes the run. Same reason as the one before the
    // optimization loop.
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // The distance-field marching pass that used to run here -- sphere tracing each band-boundary
    // edge onto d(x) = target_distance at insertion time -- is gone, as it is in 2D. Placing the
    // offset boundary is the optimization phase's job: the paper puts inserted vertices at the
    // midpoint (Sec. 5.2) and leaves the distance to Step 3, and the root find had no error
    // feedback and ran none of the inversion guards the smoother does.
    //
    // Consequence, and the reason optimize_offset() is now unconditional: conservative growth
    // leaves the band boundary on BACKGROUND-CELL boundaries, so its distance to the input complex
    // is only accurate to the local cell size until the optimization moves it.
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
    // A VERTEX ON BOTH SURFACES IS UNSATISFIABLE, not merely awkward. It sits at distance 0 from
    // the input complex, where Phi diverges, and is simultaneously required to sit on the level
    // set at distance target_distance. No placement satisfies both, so no amount of smoothing or
    // refinement can fix it -- which is why the rest of the component quietly steps around it:
    // smooth_before() refuses it in Phase B, band_vertex_is_reachable() drops it from
    // the metric, and residual_split() books it under max_pinned. Stepping around it is right for
    // the measurement and wrong as a response: it means a construction defect can sit in the mesh
    // contributing nothing but a surface that cannot be placed, and never say so.
    //
    // ON THE BOUNDING BOX IS DIFFERENT and deliberately not checked here. Such a vertex is
    // constrained but not contradictory -- it slides on the box and can still reach the level set
    // where the box permits -- and it is a legitimate outcome of growth meeting the domain edge.
    //
    // Checked after every phase, not only at construction: collapse_after_vertex() ORs the two
    // flags onto the surviving vertex, so a collapse that merges an offset vertex into an input
    // one CREATES this state out of two individually fine vertices.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input_complex) {
            continue;
        }
        // THE GEOMETRY DECIDES, NOT THE FLAGS. m_is_on_input is over-broad: the split
        // propagates it onto new vertices and the collapse ORs it onto survivors, so a
        // vertex can carry it while sitting a full target_distance from the complex --
        // which is to say sitting exactly where the offset wants it. Erroring on the flag
        // pair alone failed topological_offset_3d and _3d_edge_input at CONSTRUCTION, on
        // 134 and 119 vertices whose real distances were 0.11 to 0.20 against a
        // target_distance of 0.2: the message asserted they were at distance 0 while
        // printing distances that plainly were not.
        //
        // Unsatisfiable is a GEOMETRIC fact. smoothing_position_is_allowed() holds an
        // input-complex vertex within envelope_size of the complex, and the offset asks it
        // to reach target_distance; those two demands contradict each other only when the
        // vertex really is on the complex.
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

    // ONE WIDTH, EVERY ROUND: eps = ab_offset_envelope_rel x the Phi tolerance. Phase A is
    // pinned to the same tube whether the surface is far from the level set or nearly on it.
    //
    // This replaces a trust region that scaled eps from the previous Phase B's max residual.
    // The trust region's premise was that a far-from-converged surface needs room for
    // topological rearrangement; the cost is that Phase A then undoes Phase B's placement.
    // Measured on prism over a 16-run factorial (tau = 0.05, sequential): the trust region
    // handed round 1 an envelope of 5x tolerance, shrinking 5 -> 4.1 -> 2.8 -> 1.4 -> 0.62x,
    // against this formula's flat 0.25x -- up to 20x looser. Runs carrying it averaged 120k
    // vertices against 76k, and both of the sweep's runaway-refinement timeouts were on that
    // side. Wide early buys rearrangement freedom the loop does not need and pays for it in
    // re-placement work every round.
    //
    // Cannot be 0: refinement moves the surface by about the local chord error when a split
    // lands, and an envelope tighter than that refuses Phase A's operations outright.
    const double tol = offset_residual_tolerance();
    const double eps = std::max(m_offset_params.ab_offset_envelope_rel * tol, 1e-12);

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
    // THE CONVERGENCE CRITERION: |grad (Phi(x) - c)^2| over the offset surface -- at every band
    // vertex, and at interior samples of every band face.
    //
    // The gradient of the SAME objective Phase B's sweeps minimize for these vertices -- the
    // offset term, and nothing else. Any drift between the two would measure a different fixed
    // point than the one the sweeps converge to, which is exactly what happened while an AMIPS
    // term was carried in the criterion and not in the sweep.
    //
    // WEIGHT 1, deliberately, where the Phase B stop test used m_params.w_envelope. That test is
    // a RATIO against its own value at phase entry, so any positive constant cancels and the
    // weight was free; this one is an ABSOLUTE bound in length units, so the weight would scale
    // the bar. w_envelope is 1 - w_amips and so within 1e-4 of 1 in practice, but "within 1e-4"
    // is not a reason to let a tuning knob move a convergence threshold.
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

        // SKIPPED, not measured: a vertex the smoother declines to place this pass has a
        // gradient that is not part of the fixed point. Counted so a run cannot report
        // convergence over a band it never fully measured -- see the log line in the driver.
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

        // PINNED VERTICES ARE REPORTED, NOT GATED -- and this is the one place the gradient
        // criterion deliberately parts company with residual_split(), which counts a pinned
        // vertex toward its driving max.
        //
        // The reason is what the two numbers mean. A residual is a statement about the SURFACE:
        // a pinned vertex sitting off the level set is a real error in the offset the run
        // returns, so it belongs in the bound. A gradient is a statement about the ITERATION: it
        // asks whether the vertices the optimizer can move have stopped moving. smooth_before()
        // refuses every input-complex vertex in Phase B, so no amount of further iteration can
        // lower its gradient, and folding it into the bound would make convergence unreachable
        // by construction rather than by anything the loop did wrong. It is surfaced through
        // max_pinned instead, where the remedy is construction, not more rounds.
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

    // ... AND ACROSS THE BAND'S FACES, on the same lattice the residual is sampled on.
    //
    // A VERTEX CRITERION IS NOT A SURFACE CRITERION. Every vertex can sit exactly on the level
    // set while the triangles between them chord across it, and a vertex-only gradient reads
    // that as converged -- the same gap offset_face_samples() exists to close for the residual.
    // E = (Phi(x) - c)^2 is a field: it has a gradient at every point of space, so evaluating it
    // at a face's interior is the same computation as at a corner, not an approximation of one.
    //
    // The remedy differs, which is why the split is kept and reported. An at-vertex max is the
    // surface in the wrong PLACE and wants smoothing; an in-face max is a surface too COARSE to
    // be in the right place and wants refinement, which is Phase A's job through
    // face_criterion_rel(). Both are errors in the returned offset, so both gate.
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
    // One measurement, one definition. This used to carry its own copy of the traversal; the
    // Phase B stop test and the convergence test reading the same function is the whole point,
    // since a phase that stops on one number while the run is judged by another can sit at a
    // fixed point of neither.
    //
    // VERTICES ONLY, and that is not a drift from the convergence test -- it is the same
    // function restricted to the variables this phase owns. Phase B moves vertices and performs
    // no topological operation, so the in-face term is CONSTANT under everything it can do: a
    // chording triangle still chords after any placement of its three corners. Folding it in
    // would leave the stop test pinned at a value the sweeps cannot lower, so the phase would
    // burn ab_smooth_max_passes every round without the extra passes changing anything, and the
    // "is the cap the binding constraint?" question the pass log exists to answer would always
    // read yes. The in-face term is Phase A's to fix, through refinement.
    return gradient_split(/*include_face_samples=*/false).max_at_vertex;
}

size_t TopoOffsetTetMesh::phase_b_smooth()
{
    // SMOOTHING ONLY, TO A FIXED POINT. No topology: Phase B's single job is to move the offset
    // surface onto the level set, and the mesh it does that on is whatever Phase A left. Running
    // to convergence rather than for a fixed count is what makes the stuck check afterwards
    // meaningful -- a face still over tolerance once nothing moves is one smoothing genuinely
    // cannot place, which is a resolution problem and therefore the sizing field's business.
    std::vector<Vector3d> before(vert_capacity());

    // THE CRITERION IS THE GRADIENT, relative to its value at phase entry. Zero exactly at the
    // Gauss-Seidel fixed point, so unlike the displacement test it cannot read converged when
    // moves are blocked (a refused move has zero displacement and full gradient), and it keeps
    // going while sweeps still lower the energy. Entry-relative makes it scale-free across
    // rounds whose Phase A left very different amounts of work.
    const double g_entry = phase_b_band_gradient_linf();
    // TWO BARS, WHICHEVER COMES FIRST. The entry-relative one asks "has this phase finished the
    // work it was handed"; the ABSOLUTE one is the run's own convergence bar, and once the band
    // is under it there is nothing left for another round to do. Without the second, a phase
    // handed a large entry gradient keeps sweeping long after the run would have been declared
    // converged, because 1% of a large number is still over the bar.
    const double g_abs = offset_gradient_tolerance();
    logger().info(
        "\t[phase B] placement gradient at entry {:.6g}; the run's convergence bar is {:.6g}",
        g_entry,
        g_abs);
    if (g_entry <= 0.) {
        return 0; // already at the fixed point; nothing to smooth
    }

    // Negative ab_smooth_max_passes means UNCAPPED: run until the gradient criterion fires.
    // The only other exit is the no-progress guard below -- a gradient that has stopped
    // falling is a blocked configuration, and looping on it forever helps nobody.
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

        // TWO ORDERED SUB-SWEEPS. First place every offset-surface vertex on the level set,
        // then let every free interior vertex relax against the mesh that placement just
        // distorted. Ordered rather than interleaved so the background always relaxes against
        // the offset's FINAL position for this pass.
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

        // THE RESIDUAL PER PASS, not just once at the end of the phase.
        //
        // refine_sizing_where_phi_is_stuck() rests on "Phase B has just run to a fixed point, so
        // an offset face still over tolerance is under-resolved rather than badly placed". That
        // is only true if this loop EXITS ON the gradient criterion below rather than the pass
        // cap. Measured on specific_models/prism under the old displacement test it never did:
        // rounds ran the full ab_smooth_max_passes with displacement still 33-90x tol, so the
        // refinement was handed faces smoothing had not finished placing. This line is what
        // tells the two apart -- a residual still falling at the cap means the cap is the
        // binding constraint, a residual flat while the gradient is converged means the surface
        // is genuinely stuck and refinement is the right answer.
        //
        // max_at_vertex vs max_in_face says WHICH remedy: the surface in the wrong PLACE (wants
        // more smoothing) or too COARSE to be in the right place (wants refinement). See
        // DistanceSplit.
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
        // ONE FRAME PER SMOOTHING PASS, same debug_{N} series and same counter as the Phase A
        // passes in local_operations(), so a viewer globbing *debug_*.vtu gets both phases
        // interleaved in the order they actually ran. Opt-in through DEBUG_output_per_pass --
        // see write_optimization_debug_output(), which drops debug_ frames unless it is set --
        // because the series is large: Phase B alone ran 193 passes on the two_spheres pinch.
        // Safe to call here: write_vtu() is observational and does NOT consolidate, so the
        // frames cannot change the run they are showing.
        if (m_params.debug_output) {
            write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }
        // THE NATURAL EXIT. Every offset vertex reached its unconstrained minimum inside its own
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
        // A NON-FINITE GRADIENT IS NOT PROGRESS. Written as `g < g_prev` alone, an alternating
        // inf -> finite -> inf sequence reads as a decrease on every other pass (finite < inf is
        // true), resets this counter, and the plateau exit can never fire -- with
        // ab_smooth_max_passes < 0 that is an unbounded Phase B. Measured before this guard:
        // 772 passes at a frozen 9.477x residual, and a 362-pass run that timed out at 1800s
        // while its residual was still descending healthily.
        // BOTH measures must stall. The gradient can sit flat while the constrained count is
        // still falling -- vertices are still being freed from their one-rings even though the
        // worst placement error has not moved -- and stopping on the gradient alone would cut
        // that short. A non-finite gradient is never progress: written as `g < g_prev` alone an
        // alternating inf -> finite sequence reads as a decrease every other pass, resets this
        // counter, and with an uncapped Phase B never terminates (measured: 772 passes at a
        // frozen 9.477x residual).
        const bool g_better = std::isfinite(g) && g < g_prev;
        const bool c_better = constrained < constrained_prev;
        no_progress = (g_better || c_better) ? 0 : no_progress + 1;
        g_prev = g;
        constrained_prev = std::min(constrained_prev, constrained);
        if (no_progress >= 10) {
            // THE PLATEAU IS THE ACHIEVABLE FIXED POINT. Measured: the gradient falls 60-80x
            // from entry and then sits flat for 10 passes at 1.19x and 1.69x of an
            // entry-relative stop -- the fixed point sitting a hair above the bar, not a
            // configuration fighting the inversion guard. Nothing more is coming from
            // smoothing, so stop and let the stuck refinement answer the residual.
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

    // 3. Apply. The floor is the BAND's, as in refine_sizing_where_phi_is_stuck().
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / m_params.l);
    std::vector<size_t> changed;
    size_t n_halved = 0, n_misplaced = 0, n_ring_blocked = 0, n_at_floor = 0;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!is_band[vid]) continue;
        // MISPLACED, not under-resolved: leave it to Phase B. Counted so the two reasons a
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
    const int rounds = std::max(1, m_offset_params.ab_max_rounds);
    const int a_iters = std::max(1, m_offset_params.ab_phase_a_iterations);

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

    // TEMPORARY, for the switch-isolation sweep: every run states its own switch settings, so
    // a log is self-describing and the analysis never has to trust external bookkeeping.
    // WMTK_OFFSET_CHORD_ONLY_TRIGGER is deliberately NOT here: the only thing that reads it is
    // refine_sizing_where_phi_is_stuck(), which this loop no longer calls, so reporting it
    // would advertise a switch that cannot affect the run.
    logger().info("[experiment switches] w_amips {:g} (Phase A only)", m_params.w_amips);

    // PER-ROUND OPERATION COUNTS. iter_cnt_* are cumulative from their reset at the top of
    // optimize_offset(), so each round's contribution is the delta against the previous round's
    // totals. Phase B performs no topological operations, so a round's counts are exactly what
    // its Phase A did.
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
        // DIAGNOSTIC; see ab_no_collapse_after_first_round. Round 1 keeps its collapses because
        // the mesh as constructed genuinely needs them -- it is the ROUNDS AFTER, where every
        // split of the band is matched by a collapse removing a band vertex, that this probes.
        m_ab_collapses_disabled = m_offset_params.ab_no_collapse_after_first_round && round > 0;
        if (m_ab_collapses_disabled) {
            logger().warn("\t[phase A] COLLAPSES DISABLED (ab_no_collapse_after_first_round)");
        }
        rebuild_offset_envelope();
        mesh_improvement(a_iters);

        // ASK THE LOOP, in the loop's own units. Recomputing the bar by hand is what produced the
        // first two bugs in this driver: optimization_quality_stats() reports AMIPS against
        // optimization_stop_metric() = stop_energy, while cell_quality is AMIPS^3, so a check
        // written as cell_quality/stop_energy > 1 fails a Phase A that converged -- measured, it
        // read 4845.51 where the loop read 78.5 against 100 and had already stopped.
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        if (m_params.debug_output) {
            // NAMED BY PHASE, not by the debug counter: these two driver writes are the
            // per-phase timeline (the state each phase hands to the next), and the viewer globs
            // `*phase_*.vtu` to show exactly that series without fishing it out of the engine's
            // per-pass debug_{N} frames.
            write_optimization_debug_output(fmt::format("phase_{}A", round + 1));
        }

        // PHASE A HAS TO CONVERGE. It is TetWild on a mesh TetWild can improve, with the offset
        // surface pinned to a tolerance-wide tube; if element quality is still above stop_energy
        // when the loop gives up, something is wrong that iterating further will not fix, and
        // continuing into Phase B would optimize the offset on a mesh that cannot carry it.
        if (amips > bar) {
            // WHICH element, before saying only how bad. See log_worst_tet().
            log_worst_tet("phase A gave up");
            log_and_throw_error(
                "Phase A did not converge within {} iterations: max element quality {:.6} against "
                "stop_energy {}. The offset envelope may be too tight (ab_offset_envelope_rel "
                "{}), or the mesh has elements no operation can fix.",
                a_iters,
                amips,
                bar,
                m_offset_params.ab_offset_envelope_rel);
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

        // RECLAIM THE SLOT POOL, or the next Phase A cannot split anything at all.
        //
        // The pool is preallocated at preallocation_factor x the live count AT THE LAST
        // CONSOLIDATE, and slots consumed by operations are only returned by a consolidate --
        // not by the collapses that removed the elements. mesh_improvement() does consolidate
        // each iteration, but at TetOptimizerMesh.cpp:97, AFTER the stop test that breaks out at
        // line 89. So a Phase A that meets stop_energy on its first iteration exits without ever
        // consolidating, and every later round inherits whatever the pool was left in.
        //
        // That is not a corner case here, it is the steady state. Measured on
        // topological_offset_3d_convex: Phase A takes the mesh to 25273 vertices by splitting
        // and its collapse pass returns it to 1481, so the excursion spends slots for 25k
        // elements and frees none of them. From round 4 the split pass then reads
        //
        //     executed: 9745 | success / fail: 0 / 9745
        //     [slots] 9343 operations aborted with the preallocated slot pool exhausted
        //     #V = 1481, #T = 8131 after split
        //
        // -- EVERY split dropped, on a 1481-vertex mesh, with the sizing field asking for more
        // refinement each round and none of it possible. Quality is already under stop_energy, so
        // Phase A declares convergence after one iteration having changed nothing, Phase B smooths
        // a mesh it cannot improve, and the round repeats identically: phi bit-identical at 11.25
        // for four rounds, element quality at 40.10. The A/B loop was dead and reporting progress.
        //
        // HERE rather than at the top of Phase A, because refine_sizing_where_phi_is_stuck()
        // queues m_force_split_edges BY VERTEX ID at the end of this phase and Phase A consumes
        // them. Consolidating renumbers, so a consolidate between the queue and its consumer
        // would feed the split pass identifiers for other vertices.
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
        // THE LOOP'S CONVERGENCE TEST IS THE GRADIENT. The Phi residual and the Euclidean
        // distance error are both still measured every round and both still logged, because they
        // answer questions this one cannot -- the residual carries the chord term that ranks the
        // sizing field, the Euclidean error says how far the smoothed offset ended up from the
        // exact one -- but neither gates the run. See offset_gradient_tolerance().
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

        // BEFORE the convergence check below, so the round that converges is recorded like every
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

            // CHURN, in the same per-round shape. "recollapsed" counts split-born vertices a
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
        // the next Phase A rebuild the mesh at that resolution.
        //
        // THIS REPLACES refine_sizing_where_phi_is_stuck() RATHER THAN JOINING IT. Both only
        // ever lower a scalar, but they disagree on WHICH vertices: the old routine refines
        // around any face over tolerance, including one whose own vertex is misplaced rather
        // than under-resolved. Running both would reintroduce exactly the refinement this rule
        // withholds. The old routine is kept for now, unused by this loop.
        update_band_sizing_from_tolerance();
    }

    logger().warn(
        "A/B did not converge in {} rounds (ab_max_rounds); the offset residual is still above "
        "tolerance",
        rounds);
}

std::tuple<double, double> TopoOffsetTetMesh::optimization_quality_stats()
{
    // PHASE A IS TETWILD, so its metric is TetWild's: element quality alone, in units of
    // stop_energy, with no Phi term in the stop test, the stall test or the refinement ranking.
    // The offset is not unattended there -- m_offset_envelope holds it -- and mixing Phi back in
    // is exactly what made the two criteria fight, since a Phi that cannot improve holds the
    // metric up and keeps the stall detector firing on a mesh whose quality is still descending.
    // DELEGATED, not reimplemented. The base's version IS TetWild's, and it reports ABSOLUTE
    // AMIPS against optimization_stop_metric() = stop_energy. Returning a normalized metric here
    // instead -- 1.0 means converged, as Phase B does -- silently broke the stall refinement,
    // because refine_sizing_around_worst derives its filter from this number and then compares it
    // against cbrt(cell_quality), which is absolute: with stop_energy 10 the filter came out at
    // 100 while the worst element scored 97, so select_worst_cells returned nothing, no sizing
    // was refined and no force-split edge was ever queued. Phase A stalled at exactly 91783.4 for
    // every one of its 20 iterations with the stall detector firing 19 times and doing nothing.
    // Units are part of "identical to TetWild".
    if (m_phase == OptPhase::A) {
        return wmtk::TetOptimizerMesh::optimization_quality_stats();
    }

    // The engine's "quality" is the offset's own criterion: (max, avg) PHI RESIDUAL over the
    // reachable band, normalized so 1.0 is the tolerance. mesh_improvement() stops when the max
    // drops below optimization_stop_metric() = 1.0, i.e. exactly when the whole offset surface --
    // its vertices AND the interiors of its triangles -- is within tolerance of the level set.
    //
    // It used to be the Euclidean distance error at band VERTICES, with the average normal
    // deviation tested separately after the loop because it could not be a max-based stop. Both
    // are now one quantity: the residual is defined everywhere, so "the surface is in the wrong
    // place" and "the surface is too coarse to be in the right place" are the same measurement
    // taken at different points. The Euclidean distance is still computed and logged as a
    // diagnostic -- see compute_distance_deviation() -- because it says how far the smoothed
    // offset ended up from the exact one.
    //
    // Only the REACHABLE band counts: a pinned vertex sits on the input complex, where Phi
    // diverges, or on the domain boundary, where growth ran out of room. Leaving those in would
    // hold the metric above the bar forever -- the loop would never stop and the stall detector
    // would fire every iteration, refining around vertices nothing can help.
    const DistanceSplit r = residual_split();

    // THE RUNAWAY GUARD, before anything reports a number derived from Phi. A vertex outside the
    // support has a residual that saturates rather than growing, so the numbers below would
    // under-report it, and no smoothing move can bring it back. Checked here because this is
    // what the driver calls every iteration, and because residual_split() has already paid for
    // the measurement.
    report_outside_support("Optimization iteration", r);

    // MAX OF THE TWO CRITERIA, each over its own target, which is what 2D has always returned.
    // Phi alone left AMIPS out of the loop's stop test, its stall test and its refinement
    // ranking, so a degenerate element could sit in the mesh at MAX_ENERGY unseen and unfixed
    // for a whole run -- and the offset criterion became the only thing holding the surface
    // together, which is why weakening it decimated the surface here and not in 2D.
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
    // ONE SIZING FIELD, TWO REASONS TO REFINE IT.
    //
    // In PHASE A this is TetWildMesh::refine_sizing_around_worst verbatim -- ranked by element
    // quality, clamped the same way, seeding the same force-split edges. Phase A is TetWild, and
    // that includes how it responds to a stall; the field it writes is the same field Phase B
    // reads and writes, so the refinement each phase asks for accumulates rather than competing.
    //
    // Note the force-split half, which the offset has never had: TetWild records each worst
    // tet's LONGEST edge and split_all_edges splits exactly those, bypassing the length gate
    // and without touching the sizing field. That is how a stuck sliver gets broken up rather
    // than merely surrounded by a finer field, and its absence is measurable -- [force-split]
    // fired 0 times in every offset run to date.
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

    // THE ENGINE'S STRATEGY, DRIVEN BY THE OFFSET'S CRITERION. mesh_improvement() calls this only
    // when the metric STALLS, and this refines only around the worst-scoring offset-surface
    // faces -- TetWildMesh::refine_sizing_around_worst() with face_criterion_rel() in place of
    // AMIPS energy, which is what 2D does.
    //
    // Ranking by FACE rather than by vertex is the half that matters. A vertex score cannot see a
    // surface that has decimated to a few large triangles cutting across the offset -- every
    // vertex can sit exactly on the level set while the triangles between them do not -- so a
    // vertex-ranked field has nothing to refine and the decimation is stable. The face score
    // samples the triangle's interior, so an under-resolved face is the highest-scoring thing in
    // the mesh and is refined first.
    //
    // This replaces a per-element rule (halve every offset vertex failing sigma_max or the
    // distance band, every iteration) whose failure mode was measured on prism: sigma at a
    // genuine crease exceeds sigma_max at any resolution, so the crease bands ratcheted to the
    // floor unconditionally, gradation dragged the fine sizing into the surrounding volume, and
    // iteration 4 reached 2.8M edges. Stall-driven and worst-first, a crease that stops paying
    // for refinement stops being selected, and the ratchet stops with it.
    // THE FLOOR IS THE BAND'S, DELIBERATELY, and 3D differs from 2D here on measurement rather
    // than on principle.
    //
    // This is min_edge_length_rel x target_distance over l -- a CONSTRUCTION quantity, bounding
    // how fine the band is built -- where the Phase A branch above, and every stall refinement in
    // TetWild, TriWild and simwild, clamps against stuck_refine_min_scalar (1e-3). 2D had the
    // identical arrangement and fixing it there is what made topological_offset_2d converge, so
    // the same change was made here (fde51e7365) and then reverted, because in 3D it is not
    // justified:
    //
    //   band edges average 2.05, chord accuracy needs h < sqrt(8 R tol) = 0.89 (scalar 0.214),
    //   and this floor already permits 0.518. stuck_refine_min_scalar permits 0.0042 -- a 475x
    //   refinement nothing asked for, which drove the split pass to 43k vertices and tripled the
    //   operations dropped to slot exhaustion, 284k against 145k, for a worse final residual.
    //
    // The floor does still bind, by round 4 (refined 2 of 623, then 0 of 606, 0 of 577). It is
    // simply not what is holding 3D back -- see the note in optimize_offset_alternating() about
    // the split/collapse cycle, which is.
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
    // stuck_refine_num_worst <= 0 means NO CAP -- every face above the filter -- which is what
    // wmtk::utils::select_worst_cells does with the same parameter and is its shipped default.
    const size_t n_worst = (m_params.stuck_refine_num_worst > 0)
                               ? std::min<size_t>(scored.size(), m_params.stuck_refine_num_worst)
                               : scored.size();

    std::vector<size_t> seeds;
    seeds.reserve(3 * n_worst);
    for (size_t i = 0; i < n_worst; ++i) {
        for (const size_t v : scored[i].second) seeds.push_back(v);
    }
    // BAND-ONLY: the region growth here and the gradation below walk offset-surface vertices
    // only, instead of the full one-ring.
    //
    // The reference implementation runs its length-driven hysteresis (the todo_larger /
    // todo_smaller invariants) on the offset SURFACE child mesh, and gives the tet embedding a
    // SEPARATE, AMIPS-driven per-edge target (OffsetOptimization.cpp:1921/1963 versus
    // 1405/1481): the surface's fine sizing never touches the volume. This port shares one
    // per-vertex scalar between the two, so growing the band's refinement outward through the
    // one-ring graded it into the surrounding volume and manufactured a halo of demand nothing
    // asked for -- and with it the split/collapse churn, the slot-pool exhaustion, and the
    // starvation of the offset's own edges at the tail of the longest-first split queue.
    //
    // Measured on specific_models/prism before this, at preallocation_factor 6: every one of 80
    // Phase A iterations exhausted the slot pool (3.2M operations dropped, mean 40k/iteration),
    // element quality froze bit-stable at 166.375 against stop_energy 10 from iteration 4
    // onward, and Phase A threw without Phase B ever running. Raising preallocation_factor to
    // 50 made it worse rather than better -- the pool had been an accidental brake, and with it
    // lifted the split pass inflated 1800 -> 55000 vertices for the collapse pass to delete
    // again, net +7 vertices over 9 iterations at 7.5x the runtime.
    //
    // The volume keeps its own scalar, which also restores length_rel's meaning for it, and
    // still refines exactly as much as conformity to the band's splits forces. NOTE the Phase A
    // branch above is deliberately NOT confined: that is TetWild's own quality-driven stall
    // response over volume elements, which is the separate AMIPS-driven target the reference
    // gives the embedding, not the band's sizing leaking outward.
    const auto region = wmtk::utils::grow_vertex_region(
        seeds,
        std::max(0, m_params.stuck_refine_rings),
        [this](size_t v) { return get_one_ring_vids_for_vertex_adj(v); });

    std::vector<size_t> refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        s_floor,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });

    // NO GROWTH. The paper's Sec. 5.3.3 Step 1 raises the target length by 1.5x wherever the
    // surface is flat (sigma < sigma_min), in-band and well-shaped, and this used to do that.
    // Flatness was a normal-deviation test, which is gone; and 2D, which has run the whole
    // optimization on the shared engine for longer, carries no growth either. Monotone-down in
    // both dimensions is one fewer difference to reason about, and growth is the only direction
    // that can un-resolve a band that the criterion has just paid to resolve.
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
    // This is not a detail. The base's m_sizing_scalar defaults to 1, i.e. a target length of
    // m_params.l everywhere -- and l is a fraction of the bounding box diagonal, which on any
    // reasonable configuration is far longer than the offset surface's own edges. Starting there
    // makes every offset edge a collapse candidate on the first pass, and the offset is decimated
    // before the metrics get to speak. The per-operation normal-deviation guard cannot save it:
    // each individual collapse of a well-resolved surface barely moves the angle, so they pass one
    // at a time. In 2D skipping this was the single biggest cause of decimation.
    //
    // Starting from the current lengths says instead: keep the resolution you have, and change it
    // only where update_sizing_field() finds a reason to. The field is per-vertex, so a vertex
    // takes the mean length of its incident offset-surface edges.
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
        // EVERY vertex is seeded, not just the offset surface's. A vertex the loop above found
        // no offset face at falls back to its whole one-ring, which is the same rule applied to
        // whatever scale it happens to sit at.
        //
        // Leaving the background at the base target -- a fraction of the bounding box, which on
        // any reasonable configuration is far coarser than the mesh the construction produced --
        // is what makes the FIRST collapse pass destructive. The gate is edge length against the
        // target at its endpoints, so a background target of l against actual edges a fraction of
        // that marks essentially every interior edge as collapsible, and the pass runs before any
        // criterion has been evaluated. Seeding from the mesh as built says "keep the resolution
        // you have" everywhere, and leaves changing it to update_sizing_field(), which has a
        // reason.
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
        // that face IS offset surface -- the vertices on it are frozen and can never reach the
        // target distance, which is precisely the thing that must be measured rather than hidden.
        return cell_is_offset_band(ta);
    }
    const size_t tb = opp->tid(*this);
    const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
    if (a == b) return false; // both in the band, or neither: not the band's surface
    // The band's INNER interface, against the input complex it wraps, sits at distance 0 by
    // construction and would drag the reported error to target_distance everywhere.
    return !cell_is_input_complex(a ? tb : ta);
}


void TopoOffsetTetMesh::diag_offset_bands(const char* tag) const
{
    // OFFSET-surface edges only, against their own target l*s. The global histogram is
    // dominated by background edges, whose target is the configured length_rel scale; this is
    // the surface the optimization is actually about.
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
    // How far the offset surface actually ended up from the input complex, as a LENGTH. Exact
    // (BVH nearest point), not the straddle-edge upper bound: an input-to-offset edge can be
    // long and nearly tangential, and dhat is not a free parameter to inflate -- it selects the
    // level c, so an overestimate changes which surface the run solves for.
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
    // How far this vertex is from the level set Phi = c, as a LENGTH. The offset's own error, as
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
    // The band's Phi residual. EVERY offset-surface vertex and every face sample counts toward
    // the driving max -- including pinned vertices (on the input complex or the domain wall),
    // which used to be excluded. A pinned vertex far from the level set is a real error in the
    // offset the run returns, so hiding it reported convergence for a surface that was not at
    // target distance. The reachable/pinned split is kept as ATTRIBUTION: when the max comes
    // from a pinned vertex, the report says so, and the remedy is construction (domain size,
    // growth room), not more optimization.
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
            // THE RUNAWAY GUARD's measurement, taken here rather than in its own traversal:
            // this loop already visits exactly the vertices it cares about, and Phi is the
            // expensive part. check_offset_within_support() turns this into the error.
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
    // ... and the same measurement ACROSS the band's faces, which is what stops a surface whose
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
    // THE CUBE ROOT IS NOT OPTIONAL: cell_quality stores AMIPS^3, and stop_energy is a bar on
    // AMIPS. Without it this returned AMIPS^3 / stop_energy, so the "fails its target" test
    // score > 1 fired at AMIPS > cbrt(10) = 2.154 instead of at AMIPS > 10.
    //
    // Everything else in both dimensions already takes it: TetOptimizerMesh::quality_rel() is
    // cbrt(cell_quality(tid)) / stop_energy, get_max_avg_energy() returns cbrt(max) as the
    // reported "max energy", and 2D's TriOptimizerMesh::quality_rel() needs none because 2D
    // stores AMIPS directly. This function exists to be the 3D twin of that 2D one -- see the
    // header -- and dropping the cube root is exactly what stopped it being one.
    //
    // Measured on specific_models/prism, where Phase A converges at max AMIPS 8.5 and mean 4.0:
    // the mean face scored 4^3/10 = 6.4 and the worst 8.5^3/10 = 61, so every element of a
    // CONVERGED mesh read as failing. Since face_criterion_rel() returns the max of this and the
    // Phi residual ratio (~3.8 at worst), this term dominated and the Phi signal it is supposed
    // to sit beside never decided anything: the Phase B refinement of the time ranked faces by
    // this score and so flagged every offset face every round, force-splitting ~4300 of them and
    // taking the mesh from 117k to 402k tets between rounds, while the residual crawled
    // 5.35x -> 3.84x over five rounds. (That routine is gone -- see
    // update_band_sizing_from_tolerance() -- but face_criterion_rel() still gates collapse and
    // swap, so the cube root still matters.)
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
    // THE AMIPS TERM IS THE SAME ONE 2D RANKS BY. This used to be the Phi residual alone, and
    // that asymmetry had teeth: a degenerate element is invisible to a phi-only score, so the
    // sizing field never refines around one and the loop never sees it, while 2D -- whose score
    // starts from quality_rel() -- treats it as the worst thing in the mesh. The two dimensions
    // then needed opposite acceptance rules for the same operations, which is a symptom of
    // measuring different things rather than of the dimensions differing.
    const double tol = offset_residual_tolerance();
    double score = amips_rel_at_face(f);
    for (const size_t vid : get_face_vids(f)) {
        score = std::max(score, band_vertex_residual(vid) / tol);
    }
    // ACROSS the face as well, so a triangle too coarse to represent the offset is refined --
    // which is the mechanism that keeps the band resolved at all, and the one that replaced the
    // per-operation normal-deviation guards in collapse and swap.
    score = std::max(score, offset_face_samples(f).max / tol);
    return score;
}

void TopoOffsetTetMesh::warn_if_offset_reaches_domain_boundary() const
{
    // A band face with no opposite tet lies ON the domain boundary: the band ran out of room
    // before reaching target_distance. Counted in vertices as well as faces because the vertices
    // are what is frozen.
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
    // MECHANISM (1) OR (2)? See WallMoveStats. added-deviation moves with no tracked face at the
    // vertex are the vacuous-containment path; an invariant violation with none of them means
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

    // IS THE WALL STILL TRACKED? A face is containment-checked only if it carries
    // m_is_surface_fs -- get_surface_faces_for_vertex() filters on it -- so if the children of a
    // split wall face do not inherit the flag, the check has nothing to test and passes for a
    // vertex that has drifted anywhere at all. Counted rather than reasoned about: at init the
    // envelope holds 578 faces of which 268 are wall, and if the wall count fails to grow with
    // the mesh while the wall vertex count does, propagation is the gap.
    {
        // A COMMON wall face, not merely three vertices that each touch the box. Near a box edge
        // or corner the three vertices can sit on different walls while the triangle spans the
        // interior, and such a face is correctly untracked; only a face whose three vertices
        // SHARE a wall index actually lies in that wall and must be contained.
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

    // THE BOUNDING BOX, CHECKED POSITIONALLY rather than inferred from the accept counter.
    //
    // on_bbox_faces holds face indices k*2 (the min side of axis k) and k*2+1 (the max side),
    // and a vertex on such a face was placed by exact equality against box_min[k]/box_max[k].
    // So the invariant is exact: coordinate k must still equal that bound, bit for bit. This
    // needs no snapshot and survives consolidation, which is why it is the check rather than a
    // remembered set of positions.
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
    // WHICH vertex, and what the containment check sees when it looks at it. If it carries no
    // tracked faces the check has nothing to test; if it does, and they are inside, then the
    // envelope is not the constraint I think it is.
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
    // THE BAR IS THE ENVELOPE, NOT ZERO. This check was written while the wall was frozen, when
    // any deviation at all was a defect. The wall is now contained rather than pinned, so
    // drifting within eps is the design and only exceeding it is a violation.
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
    // GENESIS events get their own dump lane. The shared budget exists to stop a flood of
    // ordinary spike dumps, and on the round-5 explosion it was already exhausted by round-1
    // noise -- so the five genesis events, the only dumps that name the operation that created
    // an absurd tet, were never written. Genesis is rare (single digits per round), so it
    // bypasses the shared budget under its own small cap.
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
    // WHY the OFFSET's own collapse gates died this round. The base's per-gate buckets that
    // used to sit beside these are gone with the instrumentation that lived in
    // TetOptimizerMesh; so is the sizing-coarsen tally, which cannot fire now that the shared
    // collapse merges the sizing scalar with min(). Counters reset at each Phase A entry.
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

    // How many of the four carry m_is_on_region. This USED TO BE "how many the optimizer may not
    // touch at all", back when the flag froze a vertex; it no longer constrains any operation,
    // and is reported because it still says which vertices are held in the tighter envelope
    // intersection -- a tet with several is constrained, but it is no longer unfixable.
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

    // The six edges against the THREE length gates that decide this tet's fate. `split?` is the
    // split pass's candidate test; `collapse-cand?` is the SIZING-SCALED filter
    // is_weight_up_to_date applies to the collapse candidate list (4/5 l s-bar -- an edge above
    // it is never offered to collapse at all); `coarsen-gate` is the bounded coarsening pass's
    // UNSCALED bound, which is all the old line printed. Printing only the unscaled bound is
    // what hid the dead band [4/5, 4/3] l s-bar through three autopsies: at the sizing floor
    // every dump read "coarsen-gate? true (needs <= 3.3471)" while the actual candidate gate
    // sat at 0.0033 and the sliver's long edges were silently outside it.
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
    // WHICH FACE, not just how much. The vertex/face split above says whether the surface is in
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
            // essentially the same EUCLIDEAN distance is the signature of a second primitive
            // switching on -- which is not something refinement can fix. See
            // describe_active(), which names the pairs.
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
            // THE BASE'S UNION FLAG, and the one the shared operations actually read. 2D sets it
            // in label_offset_boundary(); 3D set neither this nor its counterpart at
            // init_surfaces_and_boundaries(), so m_is_on_surface was false on every vertex of the
            // mesh for the whole optimization.
            //
            // TetOptimizerMesh::is_edge_on_surface() short-circuits on it before it ever looks at
            // the face attributes, so no offset edge was ever recognised as carrying tracked
            // geometry: the collapse's surface link condition and preserve_topology (which both
            // gate on VA[..].m_is_on_surface), and the split's propagation of the flag to the new
            // vertex, were dead code for the offset. Instrumented on specific_models/prism, 0 of
            // ~1700 offset-edge split attempts per iteration were seen as surface edges; 1727 with
            // this line.
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
        "\tOffset criterion: |grad (Phi - c)^2| <= {} = convergence_gradient_norm_rel {} x "
        "target_distance "
        "{} x level-set slope^2 {:.6} -- i.e. residual <= {} ({} x target_distance) | diagnostic "
        "Phi residual bar {} (= half the gradient bound, in model units) | level c {:.6}, dhat "
        "{:.6}, {} interior samples per offset face",
        offset_gradient_tolerance(),
        m_offset_params.convergence_gradient_norm_rel,
        m_offset_params.target_distance,
        m_offset_potential->level_set_slope() * m_offset_potential->level_set_slope(),
        0.5 * m_offset_params.convergence_gradient_norm_rel * m_offset_params.target_distance,
        0.5 * m_offset_params.convergence_gradient_norm_rel,
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

    // UNCONDITIONAL, and it must stay that way: write_vtu() calls consolidate_mesh(), which
    // renumbers the mesh, which changes the order every subsequent pass enumerates operations in,
    // which changes the run. With the debug write below being the only consolidate here, turning
    // DEBUG_output on silently produced a DIFFERENT numerical result -- measured in 2D on the
    // dragon rectangle as converged-in-7 versus not-converged-in-10, identical in every other
    // parameter. Consolidating here whether or not anything is written makes the two paths agree,
    // and makes DEBUG_output what it claims to be: output only.
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // THE SHARED ENGINE'S OWN LOOP, driving the offset's convergence criterion. This used to be
    // a hand-rolled loop: one split/collapse/swap/smooth round per iteration, then halve the
    // sizing scalar at EVERY offset vertex failing a criterion, every iteration. That
    // unconditional halving is what made the runtime explode once refinement actually landed --
    // sigma at a genuine crease exceeds sigma_max at any resolution, so the crease bands
    // ratcheted to the floor regardless of whether refinement was helping, and gradation dragged
    // that fine sizing into the surrounding volume at x8 tets per halving (prism: 2.8M edges by
    // iteration 4).
    //
    // mesh_improvement() is the engine's alternative, and the offset plugs into it the same way
    // simwild does, through three virtuals:
    //   - optimization_quality_stats(): (max, avg) distance error over the band's outer
    //     vertices, normalized by convergence_target;
    //   - optimization_stop_metric(): 1.0 -- the loop stops when the worst vertex is inside the
    //     target band;
    //   - refine_sizing_around_worst(): TetWild's own quality-driven ratchet, fired only when
    //     the metric STALLS -- see the override for the details.
    // The engine consolidates every iteration and re-collects the operation queue, which is also
    // its own answer to the slot-pool exhaustion the previous hand-rolled retry loop existed
    // for: work dropped in one pass is retried in the next. The [slots] warning still reports
    // any pass that came up short.
    //
    // DISTANCE ONLY drives the loop. The convergence criterion's other half -- AVERAGE normal
    // deviation -- is an average, and the engine stops on a max, so it cannot be folded into the
    // per-vertex metric without changing what it means. It is tested after the loop instead: a
    // run can therefore exit converged-on-distance and still be reported unconverged below.
    // Deliberate, recorded, and to be revisited once distance convergence is routine.
    iter_cnt_split = 0;
    iter_cnt_split_born = 0;
    iter_cnt_recollapsed = 0;
    iter_cnt_recollapsed_same_pass = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();

    diag_offset_bands("pre");

    // FRAME 0 IS THE MESH AS CONSTRUCTED, before the optimization touches it. The shared driver
    // only writes a frame after each operation pass, so without this the timeline starts at the
    // end of the first split and there is nothing to compare against. Consumes counter 0, so the
    // driver's own frames run from 1 and the viewer's ordering is unchanged.
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
    // NO push_back HERE. op_counts is a per-A/B-round series now, recorded inside the driver
    // loop; appending the run totals as a final element would make the last entry mean something
    // different from every other one. The run total is the sum of the series.

    // Final metrics and the convergence verdict. One entry, for the whole run: the engine loop
    // owns the iterations now, so optimization_metrics is a summary rather than a history.
    //
    // ONE CRITERION, where there used to be two. The paper's termination test is a max on the
    // distance error AND an average on the normal deviation, and the asymmetry was forced by the
    // geometry: normal deviation has a floor at every sharp feature that no refinement can lower,
    // so only its average could be asked for, and an average cannot be the engine's max-based
    // stop -- which is why it was tested here, after the loop, and a run could exit
    // "converged" on distance and be reported unconverged a few lines later.
    //
    // The Phi residual has no such floor. It is defined everywhere on the surface, so the two
    // questions the old pair asked -- "is the surface in the right place" and "is it fine enough
    // to be in the right place" -- are one measurement taken at vertices and at face interiors,
    // and its max IS the engine's stop metric. See optimization_quality_stats().
    //
    // The Euclidean distance error is still computed and reported, because it says how far the
    // SMOOTHED offset ended up from the exact one -- the quantity the whole design trades away
    // at reentrant features on purpose. It is a diagnostic here, not a criterion.
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
        m_offset_params.convergence_gradient_norm_rel,
        g.max_at_vertex,
        g.max_in_face,
        g.n_face_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    // BOTH DIAGNOSTICS, neither a criterion. The Phi residual is the criterion's own quantity
    // in length units -- same vertices, same face lattice -- so it says in metres what the
    // gradient says in gradient units; the Euclidean error says how far the smoothed offset
    // ended up from the exact one, which is the quantity the design trades away at reentrant
    // features on purpose.
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
            "Converged ([max placement gradient] {} <= {} [convergence_gradient_norm_rel x "
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
        // WHICH TERM FAILED IS THE FIRST THING TO ACT ON, and naming a vertex when the max came
        // from a face interior points at the wrong remedy: an in-face max is a surface too coarse
        // to represent the level set, which wants refinement, and there is no vertex to blame for
        // it. at-vertex is the surface in the wrong place, which wants smoothing -- and only that
        // case has a worst_vid.
        if (g.max_in_face > g.max_at_vertex) {
            logger().warn(
                "Optimization did not converge ([max placement gradient] {} > {} "
                "[convergence_gradient_norm_rel x target_distance]); worst term is IN-FACE ({} vs "
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
                "[convergence_gradient_norm_rel x target_distance]); worst term is AT-VERTEX, at "
                "vertex {} "
                "({} vs {} inside faces)",
                g.max_reachable,
                gtol,
                g.worst_vid,
                g.max_at_vertex,
                g.max_in_face);
        }

        // The "blocked by topological preservation" warning that used to sit here is gone with
        // the growth pass. respect_all_topologies gated offset_tet_consistent_topology(), which
        // only grow_offset_conservative() ever called, so in 3D the flag now constrains nothing
        // and advising the user to change it would send them after a knob that cannot help.
        // (2D still honours it -- see TopoOffsetTriMesh.) If topological blocking needs a
        // diagnostic again, it has to be written against the operations that actually refuse on
        // topology now: the link conditions in split/collapse/swap.
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
    // A PLANE SLICE of the 3D potential, as a triangulated grid carrying Phi per vertex, so a
    // viewer can warp it into a height field and see the surface the optimization is actually
    // minimising against. 3D cannot write a dense volume the way 2D writes its whole domain --
    // n^3 samples of a BVH-backed potential is not affordable -- so this takes the plane through
    // the box centre with the SHORTEST extent removed, which for a model laid out along one axis
    // is the plane containing the interesting structure.
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
            // Phi diverges on the input complex, which would flatten the colour map everywhere
            // else; clamped to a few times the level, which is the range that matters. Outside
            // the support Phi is exactly 0, and that zero is meaningful -- it is the dead region
            // the placement step cannot escape from -- so it is NOT clamped away.
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

    // WRITING DEBUG OUTPUT MUST NOT CHANGE THE MESH. This used to call consolidate_mesh()
    // first, which compacts the slot arrays and RENUMBERS every vertex and cell. That makes
    // debug output non-observational: turning DEBUG_output on changes the run it is supposed to
    // be showing you. The renumbering is not cosmetic either -- under kPartition threading
    // get_partition_id() is keyed on vertex id, so which thread owns which vertex changes, and
    // with it the order operations are applied in.
    //
    // Nothing here needs the mesh compacted; it only needed the OUTPUT compacted, which is done
    // locally below instead. Point arrays stay CAPACITY-sized and slot-indexed, so
    // every vid in T/F/E stays valid and dead slots are simply unreferenced points, which VTU
    // permits. Only the cell arrays have to be packed, since a dead tid would otherwise emit a
    // (0,0,0,0) tet.
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

        // set tet tags -- row k, the PACKED index, not the slot
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

    // THE SIZING FIELD, as node data, so the result file carries what the run was asking for
    // and not only what it produced. The debug .vtu frames have always had it; the .msh had
    // not, which meant the one artifact that survives a run was the one you could not inspect
    // the field on. scripts/visualize_offset.py picks it up by this name.
    //
    // IMMEDIATELY AFTER THE VERTEX BLOCK, and it has to stay there: MshData binds an attribute
    // to entity_blocks.back(), and the per-tag loop below appends an EMPTY vertex block for
    // every group. Added after those, this would attach to an empty block.
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
