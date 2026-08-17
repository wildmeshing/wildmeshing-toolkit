
#include "TopoOffsetTetMesh.h"
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/partition_utils.hpp>

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
#include <limits>


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
    std::vector<Eigen::Vector3i> tempF;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        const auto t_opp = ff.switch_tetrahedron();
        if (!t_opp) {
            continue;
        }

        {
            const auto& tag0 = m_tet_attribute[ff.tid()].tag;
            const auto& tag1 = m_tet_attribute[t_opp.value().tid()].tag;
            if (tag0 == tag1) {
                continue;
            }
        }

        m_face_attribute[ff.fid()].m_is_surface_fs = true;

        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();
        m_vertex_extra[v1].m_is_on_input = true;
        m_vertex_extra[v2].m_is_on_input = true;
        m_vertex_extra[v3].m_is_on_input = true;
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
        logger().info("Init envelope from tet tags");
        // build envelopes
        std::vector<Eigen::Vector3d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        m_V_envelope = tempV;
        m_F_envelope = tempF;
        m_envelope = std::make_shared<SampleEnvelope>();
        m_envelope->use_exact = true;
        m_envelope->init(m_V_envelope, m_F_envelope, m_envelope_eps);
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

bool TopoOffsetTetMesh::is_edge_on_input(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_extra[v1_id].m_is_on_input || !m_vertex_extra[v2_id].m_is_on_input) return false;

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
        if (face_is_input(fid)) return true;
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
    return m_vertex_extra.at(vid).m_is_on_input || m_vertex_extra.at(vid).m_is_on_offset;
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
    m_edge_split_mode = EdgeSplitMode::Initial;
    marching_tets();
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // run BFS
    grow_offset_conservative();
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

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

std::tuple<double, double> TopoOffsetTetMesh::optimization_quality_stats()
{
    // The engine's "quality" is the offset's distance criterion: (max, avg) distance error over
    // the band's outer vertices, normalized so 1.0 is the convergence target. mesh_improvement()
    // stops when the max drops below optimization_stop_metric() = 1.0, i.e. exactly when the
    // worst-placed offset vertex enters the target band.
    //
    // DISTANCE ONLY -- the criterion's average-normal-deviation half cannot be a max-based stop
    // and is tested after the loop in optimize_offset(); see the comment there. Kept cheap and
    // side-effect-light because the engine calls this after every operation pass.
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const double ct = std::max(m_offset_params.convergence_target, 1e-16);
    return {max_dist / ct, avg_dist / ct};
}

size_t TopoOffsetTetMesh::refine_sizing_around_worst(double)
{
    // THE ENGINE'S STRATEGY, DRIVEN BY THE OFFSET'S CRITERION. mesh_improvement() calls this only
    // when the metric STALLS, and this refines only around the worst-placed band vertices --
    // TetWildMesh::refine_sizing_around_worst() with the offset's distance error in place of
    // AMIPS energy.
    //
    // This replaces a per-element rule (halve every offset vertex failing sigma_max or the
    // distance band, every iteration) whose failure mode was measured on prism: sigma at a
    // genuine crease exceeds sigma_max at any resolution, so the crease bands ratcheted to the
    // floor unconditionally, gradation dragged the fine sizing into the surrounding volume, and
    // iteration 4 reached 2.8M edges. Stall-driven and worst-first, a crease that stops paying
    // for refinement stops being selected, and the ratchet stops with it.
    const double l = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    const double ct = std::max(m_offset_params.convergence_target, 1e-16);

    // The band's outer vertices, by the rule compute_distance_deviation() uses.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }

    const auto dist_rel = [&](size_t vid) {
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double d = (p - m_input_complex_bvh.nearest_point(p)).norm();
        return std::abs(d - m_offset_params.target_distance) / ct;
    };

    // Worst band vertices above the criterion (filter 1.0: a vertex already inside the target
    // band is never a reason to refine), capped at stuck_refine_num_worst.
    const auto worst = wmtk::utils::select_worst_cells(
        vert_capacity(),
        [&](size_t vid) { return on_band[vid]; },
        dist_rel,
        1.0,
        m_params.stuck_refine_num_worst);
    if (worst.empty()) {
        return 0;
    }

    std::vector<size_t> seeds;
    seeds.reserve(worst.size());
    for (const auto& [_, vid] : worst) {
        seeds.push_back(vid);
    }
    // BAND-ONLY: the reference keeps its length-driven hysteresis on the offset
    // SURFACE child mesh and never lets its sizing leak into the embedding, whose target
    // is separate and AMIPS-driven (OffsetOptimization.cpp:1921/1963 vs 1405/1481).
    // Spreading the band's fine sizing into the volume is what manufactured the halo
    // demand -- and with it the churn, the slot exhaustion and the split-queue
    // starvation. So the region growth and the gradation below walk BAND vertices only;
    // the background keeps its scalar, which also restores length_rel's meaning for the
    // volume.
    const auto band_ring = [&](size_t v) {
        std::vector<size_t> out;
        for (const size_t nb : get_one_ring_vids_for_vertex(v)) {
            if (on_band[nb]) out.push_back(nb);
        }
        return out;
    };
    const auto region =
        wmtk::utils::grow_vertex_region(seeds, std::max(0, m_params.stuck_refine_rings), band_ring);

    std::vector<size_t> refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        s_floor,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });

    // GROWTH, kept from the paper (Sec. 5.3.3, Step 1: "if shape regularity is [above] 0.5 and
    // the normal deviation is [below] sigma_min, we increase the target length by 1.5") and
    // deliberately NOT part of the engine's monotone-down strategy -- recorded as a decision to
    // revisit. It now runs on stall rather than every iteration, which is a change of frequency,
    // not of rule. The guards are the old rule's: grow only where the vertex is inside the
    // distance band, flat (sigma < sigma_min) and well-shaped, so growth can never fight the
    // refinement above.
    {
        const double sigma_min = m_offset_params.min_normal_deviation_deg;
        std::vector<double> min_mrm(vert_capacity(), std::numeric_limits<double>::max());
        for (const Tuple& f : get_faces()) {
            if (!face_is_offset_surface_live(f)) continue;
            const auto vs = get_face_vids(f);
            const double mrm = mean_ratio_metric(
                m_vertex_attribute[vs[0]].m_posf,
                m_vertex_attribute[vs[1]].m_posf,
                m_vertex_attribute[vs[2]].m_posf);
            for (const size_t v : vs) {
                min_mrm[v] = std::min(min_mrm[v], mrm);
            }
        }
        for (size_t vid = 0; vid < vert_capacity(); ++vid) {
            if (!on_band[vid]) continue;
            if (region.count(vid)) continue; // just refined; growing it back would be a tug-of-war
            if (dist_rel(vid) > 1.) continue;
            if (min_mrm[vid] < m_offset_params.sizing_mrm_threshold) continue;
            if (max_offset_surface_normal_deviation_at_vertex(vid) >= sigma_min) continue;
            double& sc = m_vertex_attribute[vid].m_sizing_scalar;
            sc = std::clamp(sc * 1.5, s_floor, m_offset_params.max_sizing_scalar);
        }
    }

    wmtk::utils::gradation_smooth_sizing(
        m_offset_params.sizing_gradation,
        refined,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        band_ring);

    logger().info(
        "[stuck-refine] worst {} band vertices (worst dist {:.4}x target), refined {} of {} "
        "region vertices",
        worst.size(),
        worst.empty() ? 0. : worst.back().first,
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
    int n_seeded = 0;
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
        if (n == 0) continue; // not on the offset surface: the background keeps the base target

        raw_sum += sum_len / n;
        ++n_seeded;
        m_vertex_attribute[vid].m_sizing_scalar =
            std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
    }
    logger().info(
        "\tOffset sizing seed: {} vertices, mean incident length {:.6} (base l {:.6}, l_min {:.6} "
        "= 2*{}*sin({} deg), scalar floor {:.6})",
        n_seeded,
        n_seeded > 0 ? raw_sum / n_seeded : 0.,
        l,
        m_offset_params.min_edge_length,
        m_offset_params.target_distance,
        m_offset_params.max_normal_deviation_deg,
        s_floor);
}

bool TopoOffsetTetMesh::offset_field_normal(const Vector3d& p, Vector3d& n) const
{
    const Vector3d nearest = m_input_complex_bvh.nearest_point(p);
    const Vector3d diff = p - nearest;
    const double d = diff.norm();
    if (d < 1e-12) return false; // on the complex: the field has no direction here
    n = diff / d;
    return true;
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

std::pair<double, double> TopoOffsetTetMesh::compute_normal_deviation() const
{
    double max_nd = 0., sum_nd = 0.;
    size_t n = 0;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const double nd = face_normal_deviation(f);
        max_nd = std::max(max_nd, nd);
        sum_nd += nd;
        ++n;
    }
    return {max_nd, n > 0 ? sum_nd / n : 0.};
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
            n_input_f += face_is_input(face_tuple.fid(*this));
            n_bbox_f += !face_tuple.switch_tetrahedron(*this).has_value();
        }
    }
    const auto& ve = m_vertex_extra[vid];
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
        ve.m_is_on_input,
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
    //   - refine_sizing_around_worst(): ratchet the sizing down ONLY around the worst-placed
    //     vertices, and only when the metric STALLS -- see the override for the details.
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
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    iter_cnt_collapse_nd_reject = 0;
    iter_cnt_swap_nd_reject = 0;
    m_smooth_trace.reset();

    diag_offset_bands("pre");
    mesh_improvement(m_offset_params.optimization_iterations);

    // Cumulative over the whole run, not per iteration as before: the engine loop has no
    // per-iteration hook, and the per-pass numbers it logs itself carry the history.
    log_smooth_trace();
    logger().info(
        "splits = {}  |  collapses = {} ({} refused by normal deviation, {} removed an offset "
        "vertex)  |  swaps = {} ({} refused by normal deviation)",
        iter_cnt_split.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_nd_reject.load(),
        iter_cnt_collapse_offset_removed.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_nd_reject.load());
    op_counts.push_back({{iter_cnt_split.load(), iter_cnt_collapse.load(), iter_cnt_swap.load()}});

    // Final metrics and the convergence verdict. One entry, for the whole run: the engine loop
    // owns the iterations now, so optimization_metrics is a summary rather than a history.
    //
    // The asymmetry -- max for distance, average for angle -- is the paper's (Sec. 5.3,
    // Termination), and it is forced by the geometry: distance error can genuinely go to zero
    // everywhere, so the max is a fair bar, while normal deviation has a floor at every sharp
    // feature that no refinement can lower, so only its average can be asked for.
    // convergence_normal_deviation <= 0 disables the angular half entirely.
    diag_offset_bands("final");
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const auto [max_norm, avg_norm] = compute_normal_deviation();
    logger().info(
        "max dist err: {} | avg dist err: {} | max normal dev: {} | avg normal dev: {}",
        max_dist,
        avg_dist,
        max_norm,
        avg_norm);
    optimization_metrics.push_back({{max_dist, avg_dist, max_norm, avg_norm}});
    log_worst_dist_vertex();

    const bool dist_ok = max_dist <= m_offset_params.convergence_target;
    const bool norm_ok = m_offset_params.convergence_normal_deviation <= 0. ||
                         avg_norm <= m_offset_params.convergence_normal_deviation;
    m_converged = dist_ok && norm_ok;
    if (m_converged) {
        if (m_offset_params.convergence_normal_deviation > 0.) {
            logger().info(
                "Converged ([max_dist] {} <= {} [convergence_target], [avg_normal_dev] {} <= "
                "{} [convergence_normal_deviation])",
                max_dist,
                m_offset_params.convergence_target,
                avg_norm,
                m_offset_params.convergence_normal_deviation);
        } else {
            logger().info(
                "Converged ([max_dist] {} <= {} [convergence_target], normal deviation "
                "criterion disabled)",
                max_dist,
                m_offset_params.convergence_target);
        }
    }

    if (!m_converged && !optimization_metrics.empty()) {
        // Name the criterion that actually failed; with two of them, reporting only the distance
        // sends you looking at the wrong one.
        const double max_dist = optimization_metrics.back()[0];
        const double avg_norm = optimization_metrics.back()[3];
        if (max_dist > m_offset_params.convergence_target) {
            logger().warn(
                "Optimization did not converge ([max_dist] {} > {} [convergence_target])",
                max_dist,
                m_offset_params.convergence_target);
        }
        if (m_offset_params.convergence_normal_deviation > 0. &&
            avg_norm > m_offset_params.convergence_normal_deviation) {
            logger().warn(
                "Optimization did not converge ([avg_normal_dev] {} > {} "
                "[convergence_normal_deviation])",
                avg_norm,
                m_offset_params.convergence_normal_deviation);
        }

        // Is topological preservation holding the offset back, as opposed to a handful of
        // over-constrained vertices? See TOPOLOGY_BLOCK_AVG_FRAC for why the average is the
        // discriminator.
        const double avg_dist = optimization_metrics.back()[1];
        if (m_offset_params.respect_all_topologies &&
            avg_dist > TOPOLOGY_BLOCK_AVG_FRAC * m_offset_params.target_distance) {
            logger().warn(
                "Offset growth appears to be BLOCKED BY TOPOLOGICAL PRESERVATION: "
                "respect_all_topologies is true and the AVERAGE distance error is {:.1f}% of "
                "target_distance ({} vs {}), against max {:.1f}%. Error that uniform means whole "
                "regions of the offset never reached the target distance, not that a few vertices "
                "are over-constrained; with respect_all_topologies every tag's topology is frozen "
                "after offset initialization, so the operations that would let the offset advance "
                "through a narrow region are refused and it stays where conservative growth left "
                "it. Set respect_all_topologies false if only the offset's own topology matters, "
                "or reduce target_distance so the offset does not need to pass through those "
                "regions.",
                100.0 * avg_dist / m_offset_params.target_distance,
                avg_dist,
                m_offset_params.target_distance,
                100.0 * max_dist / m_offset_params.target_distance);
        }
    }

    // Escalate to a hard failure if the caller asked for it, AFTER the warnings above so the log
    // still names which criterion missed before the throw. Guarded on !m_converged alone rather
    // than on the block above: a run that produced no metrics at all has certainly not converged.
    if (!m_converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above for the criterion that failed.",
            optimization_metrics.size(),
            m_offset_params.optimization_iterations);
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


void TopoOffsetTetMesh::grow_offset_conservative()
{
    std::queue<Tuple> tets_q;
    auto all_tets = get_tets();

    for (const Tuple& t : all_tets) {
        size_t t_id = t.tid(*this);
        if ((m_tet_attribute[t_id].label == 0) && (offset_tet_consistent_topology(t_id))) {
            tets_q.push(t);
        }
    }
    logger().info("\tConservative grow: Initial queue size {}", tets_q.size());

    while (!tets_q.empty()) {
        Tuple curr_tet = tets_q.front();
        tets_q.pop();

        size_t tet_id = curr_tet.tid(*this);
        if (m_tet_attribute[tet_id].label != 0) { // already in offset
            continue;
        }

        // ensure tet wouldn't change topology
        if ((m_tet_attribute[tet_id].label != 0) || (!offset_tet_consistent_topology(tet_id))) {
            continue;
        }

        bool in_offset = tet_is_in_offset_conservative(
            tet_id,
            m_offset_params.relative_ball_threshold * m_offset_params.target_distance);
        if (in_offset) {
            m_tet_attribute[tet_id].label = 2;
            for (int i = 0; i < 4; i++) { // propagate label to faces
                size_t f_id = tuple_from_face(tet_id, i).fid(*this);
                if (m_face_extra[f_id].label != 1) {
                    m_face_extra[f_id].label = 2;
                }
            }
            for (int i = 0; i < 6; i++) {
                size_t e_id = tuple_from_edge(tet_id, i).eid(*this);
                if (m_edge_attribute[e_id].label != 1) {
                    m_edge_attribute[e_id].label = 2;
                }
            }
            auto vs = oriented_tet_vids(tet_id);
            for (const size_t& v_id : vs) {
                if (m_vertex_extra[v_id].label != 1) {
                    m_vertex_extra[v_id].label = 2;
                }
            }

            // collect adjacent tets, add to queue
            auto adj_tets = get_face_adjacent_tets(curr_tet);
            for (const Tuple& t : adj_tets) {
                if (m_tet_attribute[t.tid(*this)].label != 0) {
                    continue;
                }
                tets_q.push(t);
            }
        }
    }
}

void TopoOffsetTetMesh::grow_offset_aggressive()
{
    std::queue<Tuple> tets_q;
    auto all_tets = get_tets();

    for (const Tuple& t : all_tets) {
        size_t t_id = t.tid(*this);
        if ((m_tet_attribute[t_id].label == 0) && (offset_tet_consistent_topology(t_id))) {
            tets_q.push(t);
        }
    }
    logger().info("\tAggressive grow: Initial queue size {}", tets_q.size());

    while (!tets_q.empty()) {
        Tuple curr_tet = tets_q.front();
        tets_q.pop();

        size_t tet_id = curr_tet.tid(*this);
        if (m_tet_attribute[tet_id].label != 0) { // already in offset
            continue;
        }

        // ensure tet wouldn't change topology
        if ((m_tet_attribute[tet_id].label != 0) || (!offset_tet_consistent_topology(tet_id))) {
            continue;
        }

        bool in_offset = tet_is_in_offset_aggressive(tet_id);
        if (in_offset) {
            m_tet_attribute[tet_id].label = 2;
            for (int i = 0; i < 4; i++) { // propagate label to faces
                size_t f_id = tuple_from_face(tet_id, i).fid(*this);
                if (m_face_extra[f_id].label != 1) {
                    m_face_extra[f_id].label = 2;
                }
            }
            for (int i = 0; i < 6; i++) {
                size_t e_id = tuple_from_edge(tet_id, i).eid(*this);
                if (m_edge_attribute[e_id].label != 1) {
                    m_edge_attribute[e_id].label = 2;
                }
            }
            auto vs = oriented_tet_vids(tet_id);
            for (const size_t& v_id : vs) {
                if (m_vertex_extra[v_id].label != 1) {
                    m_vertex_extra[v_id].label = 2;
                }
            }

            // collect adjacent tets, add to queue
            auto adj_tets = get_face_adjacent_tets(curr_tet);
            for (const Tuple& t : adj_tets) {
                if (m_tet_attribute[t.tid(*this)].label != 0) {
                    continue;
                }
                tets_q.push(t);
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


void TopoOffsetTetMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu (tag for offset is included)", path);

    consolidate_mesh();
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
    MatrixXi T(tet_capacity(), 4);
    MatrixXi F_in(faces_in.size(), 3);
    MatrixXi F_off(faces_off.size(), 3);
    MatrixXi E(edges.size(), 2);

    V.setZero();
    T.setZero();
    F_in.setZero();
    F_off.setZero();
    E.setZero();

    // last matrix is offset
    std::vector<MatrixXd> tags(m_tags_count + 1, MatrixXd(tet_capacity(), 1));
    VectorXd labels(vert_capacity());
    labels.setZero();
    VectorXd v_order(vert_capacity());
    v_order.setZero();
    VectorXd v_id(vert_capacity());
    v_id.setZero();
    VectorXd v_sizing(vert_capacity());
    v_sizing.setZero();

    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);

        // set tet tags
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](t_id, 0) = (m_tet_attribute[t_id].tag.count(i) == 1) ? 1 : 0;
        }
        tags[m_tags_count](t_id, 0) = (m_tet_attribute[t_id].label == 2) ? 1 : 0;
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

    // set tet verts
    for (const Tuple& t : tets) {
        // set tet verts
        const auto& loc_vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(t.tid(*this), j) = loc_vs[j].vid(*this);
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
