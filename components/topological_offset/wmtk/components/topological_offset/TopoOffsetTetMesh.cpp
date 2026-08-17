
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
}


void TopoOffsetTetMesh::init_offset_potential()
{
    if (m_phi_V.rows() == 0) {
        log_and_throw_error("init_offset_potential() called before init_input_complex_bvh()");
    }
    m_offset_potential = std::make_shared<OffsetPotential3D>(
        m_phi_V,
        m_phi_E,
        m_phi_F,
        m_phi_P,
        m_offset_params.target_distance,
        m_offset_params.offset_dhat_factor);
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

void TopoOffsetTetMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A VERTEX ON BOTH SURFACES IS UNSATISFIABLE, not merely awkward. It sits at distance 0 from
    // the input complex, where Phi diverges, and is simultaneously required to sit on the level
    // set at distance target_distance. No placement satisfies both, so no amount of smoothing or
    // refinement can fix it -- which is why the rest of the component quietly steps around it:
    // smoothing_extra_energy() gives it no offset term, band_vertex_is_reachable() drops it from
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
        if (m_vertex_extra[vid].m_is_on_offset && m_vertex_extra[vid].m_is_on_input) {
            both.push_back(vid);
        }
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

    // ONE PHI TOLERANCE by default, so Phase A may move the offset surface by exactly as much as
    // the convergence test is willing to ignore. Tighter and Phase A cannot improve the elements
    // straddling the surface at all; looser and it can undo a Phi that Phase B had already
    // brought inside tolerance.
    const double eps =
        std::max(m_offset_params.ab_offset_envelope_rel * offset_residual_tolerance(), 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>();
    m_offset_envelope->use_exact = true;
    m_offset_envelope->init(V, F, eps);
    logger().info(
        "\t[phase A] offset envelope rebuilt: {} faces, eps {:.6g} ({:.4}x the Phi tolerance)",
        F.size(),
        eps,
        m_offset_params.ab_offset_envelope_rel);
}

size_t TopoOffsetTetMesh::phase_b_smooth()
{
    // SMOOTHING ONLY, TO A FIXED POINT. No topology: Phase B's single job is to move the offset
    // surface onto the level set, and the mesh it does that on is whatever Phase A left. Running
    // to convergence rather than for a fixed count is what makes the stuck check afterwards
    // meaningful -- a face still over tolerance once nothing moves is one smoothing genuinely
    // cannot place, which is a resolution problem and therefore the sizing field's business.
    std::vector<Vector3d> before(vert_capacity());

    size_t pass = 0;
    double disp = 0.;
    for (; pass < size_t(std::max(1, m_offset_params.ab_smooth_max_passes)); ++pass) {
        for (size_t i = 0; i < vert_capacity(); ++i) {
            before[i] = m_vertex_attribute[i].m_posf;
        }

        smooth_all_vertices(1);

        disp = 0.;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            disp = std::max(disp, (m_vertex_attribute[vid].m_posf - before[vid]).norm());
        }
        // Relative to the target edge length, so the test means the same thing at any scale.
        const double tol = m_offset_params.ab_smooth_tol * std::max(m_params.l, 1e-16);
        logger().info(
            "\t[phase B] pass {}: max vertex displacement {:.6g} (tol {:.6g})",
            pass + 1,
            disp,
            tol);
        if (disp <= tol) {
            ++pass;
            break;
        }
    }
    return pass;
}

size_t TopoOffsetTetMesh::refine_sizing_where_phi_is_stuck()
{
    // WHAT SMOOTHING COULD NOT PLACE. Phase B has just run to a fixed point, so an offset face
    // still over tolerance is not badly placed, it is under-resolved: the three vertices are
    // where the energy wants them and the triangle between them still cuts across the level set.
    // That is the one situation refinement actually answers, which is why this is the only place
    // Phi is allowed to drive the sizing field -- unlike the joint loop, where the field was
    // refined whenever the combined metric stalled, which was nearly every iteration regardless
    // of whether Phi needed resolution.
    const double tol = offset_residual_tolerance();
    std::vector<std::pair<double, std::array<size_t, 3>>> stuck;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const double score = face_criterion_rel(f);
        if (score <= 1.0) continue;
        stuck.emplace_back(score, get_face_vids(f));
    }
    if (stuck.empty()) {
        return 0;
    }
    std::sort(stuck.begin(), stuck.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });
    const size_t n_worst = (m_params.stuck_refine_num_worst > 0)
                               ? std::min<size_t>(stuck.size(), m_params.stuck_refine_num_worst)
                               : stuck.size();

    std::vector<size_t> seeds;
    seeds.reserve(3 * n_worst);
    for (size_t i = 0; i < n_worst; ++i) {
        for (const size_t v : stuck[i].second) seeds.push_back(v);
    }

    // FORCE-SPLIT THE STUCK FACES' LONGEST EDGES, and this is not optional -- it is the half of
    // TetWild's stall response that lowering a scalar cannot substitute for.
    //
    // Lowering a sizing scalar only PERMITS a split next pass; the length gate still has to agree,
    // and once the scalar is at the floor it never will. Force-split MAKES the split, once, gate
    // or no gate. Measured on topological_offset_3d_convex before this: the refinement decayed
    // from 730 of 742 region vertices in round 1 to 2 of 675 by round 4 and 0 of 675 thereafter --
    // a Phase B response that had become a literal no-op -- while the residual sat at 10.98,
    // 10.99, 10.99, 10.99. That plateau was read as evidence of a re-triangulation ratchet in
    // Phase A. It was not; it was a dead refinement path. 2D, which carries the same two defects
    // ported from here, stalled between 2.0x and 2.7x for eight rounds and converged once both
    // were fixed.
    //
    // THE LONGEST EDGE OF A FACE, not the face itself, because a face is not splittable and this
    // is TetWild's own convention for the same situation. 2D queues its stuck simplex directly:
    // there the offset IS a polyline, so the stuck thing already is an edge.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (size_t i = 0; i < n_worst; ++i) {
            m_force_split_edges.insert(
                wmtk::utils::longest_edge(stuck[i].second, [this](size_t v) -> const Vector3d& {
                    return m_vertex_attribute[v].m_posf;
                }));
        }
    }

    const auto region = wmtk::utils::grow_vertex_region(
        seeds,
        std::max(0, m_params.stuck_refine_rings),
        [this](size_t v) { return get_one_ring_vids_for_vertex(v); });

    // THE FLOOR IS THE OPTIMIZATION'S, NOT THE CONSTRUCTION'S -- see the identical note in the
    // Phase B branch of refine_sizing_around_worst().
    const double s_floor = m_params.stuck_refine_min_scalar;
    const auto refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        s_floor,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
    wmtk::utils::gradation_smooth_sizing(
        m_offset_params.sizing_gradation,
        refined,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex(v); });

    logger().info(
        "\t[phase B] {} of {} offset faces stuck over tolerance (worst {:.4}x), {} force-split, "
        "refined {} of {} region vertices (floor {:.4})",
        n_worst,
        stuck.size(),
        stuck.front().first,
        m_force_split_edges.size(),
        refined.size(),
        region.size(),
        s_floor);
    return refined.size();
}

void TopoOffsetTetMesh::optimize_offset_alternating()
{
    const int rounds = std::max(1, m_offset_params.ab_max_rounds);
    const int a_iters = std::max(1, m_offset_params.ab_phase_a_iterations);

    // Before anything runs, so a construction defect is reported as one rather than surfacing
    // later as a residual that will not converge.
    check_no_vertex_on_both_surfaces("construction");

    for (int round = 0; round < rounds; ++round) {
        // ---- PHASE A: TetWild, with the offset held inside its envelope ----
        logger().info("======== A/B round {} / {}: phase A ========", round + 1, rounds);
        m_phase = OptPhase::A;
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
            write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }

        // PHASE A HAS TO CONVERGE. It is TetWild on a mesh TetWild can improve, with the offset
        // surface pinned to a tolerance-wide tube; if element quality is still above stop_energy
        // when the loop gives up, something is wrong that iterating further will not fix, and
        // continuing into Phase B would optimize the offset on a mesh that cannot carry it.
        if (amips > bar) {
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

        const size_t passes = phase_b_smooth();

        const DistanceSplit r = residual_split();
        report_outside_support("A/B round", r);
        const double phi = r.max_reachable / offset_residual_tolerance();
        logger().info(
            "\t[phase B] {} smoothing passes, max Phi residual {:.4}x tolerance",
            passes,
            phi);
        if (m_params.debug_output) {
            write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }

        if (phi <= 1.0) {
            logger().info(
                "A/B converged after {} round(s): amips {:.4}x, phi {:.4}x",
                round + 1,
                amips,
                phi);
            return;
        }

        // Not converged: refine where smoothing could not place the surface, and let the next
        // Phase A rebuild the mesh at that resolution.
        refine_sizing_where_phi_is_stuck();
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
    // THE FLOOR IS THE OPTIMIZATION'S, NOT THE CONSTRUCTION'S. stuck_refine_min_scalar (1e-3) is
    // what the Phase A branch above clamps against, and what every stall refinement in TetWild,
    // TriWild and simwild clamps against.
    //
    // min_edge_length is a different quantity for a different phase: it is derived from
    // min_edge_length_rel x target_distance and bounds how fine the BAND IS BUILT. As an
    // optimization floor it caps every band triangle at roughly half the offset distance, and a
    // triangle that coarse across a curved level set misses it by about the tolerance wherever
    // its vertices sit. On topological_offset_3d_convex it works out to 0.1237 against
    // stuck_refine_min_scalar's 0.001 -- 124x -- and the two phases of one loop were clamping
    // against different floors, which is how the discrepancy survived.
    const double s_floor = m_params.stuck_refine_min_scalar;

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
    const auto region = wmtk::utils::grow_vertex_region(
        seeds,
        std::max(0, m_params.stuck_refine_rings),
        [this](size_t v) { return get_one_ring_vids_for_vertex(v); });

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
    wmtk::utils::gradation_smooth_sizing(
        m_offset_params.sizing_gradation,
        refined,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex(v); });

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
    FaceSamples s;
    const int k = m_offset_params.offset_residual_samples;
    if (k <= 0) return s;

    const auto vs = get_face_vids(f);
    for (const size_t v : vs) {
        if (!band_vertex_is_reachable(v)) return s;
    }
    const Vector3d p0 = m_vertex_attribute[vs[0]].m_posf;
    const Vector3d p1 = m_vertex_attribute[vs[1]].m_posf;
    const Vector3d p2 = m_vertex_attribute[vs[2]].m_posf;

    // Interior lattice points of the barycentric subdivision at denominator n = k + 2: every
    // (i, j, l) with i + j + l = n and each >= 1, so nothing lands on an edge or a vertex (those
    // are measured separately, and a sample ON a vertex would double-count it).
    //
    // n = k + 2 rather than 2D's k + 1 because a triangle has no interior lattice point at
    // denominator 2. The counts are 1, 3, 6, 10 for k = 1, 2, 3, 4, so k keeps its meaning as
    // "sampling density" and k = 1 is the centroid -- which is the sample that matters most,
    // since a triangle inscribed in the offset is furthest from it at its centroid.
    const int n = k + 2;
    for (int i = 1; i < n; ++i) {
        for (int j = 1; j < n - i; ++j) {
            const int l = n - i - j;
            if (l < 1) continue;
            const Vector3d q = (double(i) * p0 + double(j) * p1 + double(l) * p2) / double(n);
            const double r = m_offset_potential->residual_length(q);
            s.max = std::max(s.max, r);
            s.sum += r;
            ++s.n;
        }
    }
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
    // The band's Phi residual, split by whether the optimizer can do anything about it.
    //
    // REACHABLE: an offset-surface vertex free to be placed on the level set. PINNED: one that
    // cannot be, whatever the optimizer does -- it lies on the input complex, where Phi diverges
    // and the distance is 0 by definition, or on the domain boundary, where conservative growth
    // ran out of room. Neither is an optimization failure and neither can be improved by
    // refining around it, so only the reachable half drives the loop and the sizing field.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double err = m_offset_potential->residual_length(p);
        if (band_vertex_is_reachable(vid)) {
            s.max_reachable = std::max(s.max_reachable, err);
            s.max_at_vertex = std::max(s.max_at_vertex, err);
            sum_reachable += err;
            ++s.n_reachable;
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
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    // ... and the same measurement ACROSS the band's faces, which is what stops a surface whose
    // vertices sit on the level set but whose triangles cut across it from reading as converged.
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const FaceSamples fs = offset_face_samples(f);
        if (fs.n == 0) continue; // no samples asked for, or an unreachable corner
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
    return cell_quality(tid) / std::max(m_params.stop_energy, 1e-16);
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
        if (!band_vertex_is_reachable(vid)) continue;
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

    // The offset surface as CONSTRUCTED must already be inside the potential's support, or
    // nothing the optimization does can move it. Checked before any operation runs so that a
    // construction defect is reported as one.
    check_offset_within_support("Offset as constructed");

    logger().info(
        "\tOffset criterion: Phi residual <= {} = offset_residual_rel {} x target_distance {} "
        "| level c {:.6}, dhat {:.6}, {} interior samples per offset face",
        offset_residual_tolerance(),
        m_offset_params.offset_residual_rel,
        m_offset_params.target_distance,
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
        "splits = {} (offset-edge: {} offered, {} frozen, {} refused by the base's before-gate; "
        "{} reached after_cells with both endpoints on-offset; {} splits produced an "
        "on-offset vertex; {} splits refused in after)  |  "
        "collapses = {} ({} "
        "removed an offset vertex, {} refused by the offset criterion)  |  swaps = {} ({} "
        "refused by the offset criterion)",
        iter_cnt_split.load(),
        iter_cnt_split_offset_before.load(),
        iter_cnt_split_offset_frozen.load(),
        iter_cnt_split_offset_base_reject.load(),
        iter_cnt_split_offset_endpoints.load(),
        iter_cnt_split_offset.load(),
        iter_cnt_split_offset_reject.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_offset_removed.load(),
        iter_cnt_collapse_offset_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_offset_reject.load());
    op_counts.push_back({{iter_cnt_split.load(), iter_cnt_collapse.load(), iter_cnt_swap.load()}});

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
    const double tol = offset_residual_tolerance();
    logger().info(
        "phi residual: max {} (avg {}) vs tolerance {} = {} x target_distance | at vertices {}, "
        "inside faces {} | {} reachable samples, {} pinned vertices || euclid dist err: max {} | "
        "avg {}",
        r.max_reachable,
        r.avg_reachable,
        tol,
        m_offset_params.offset_residual_rel,
        r.max_at_vertex,
        r.max_in_face,
        r.n_reachable,
        r.n_pinned,
        max_dist,
        avg_dist);
    optimization_metrics.push_back({{max_dist, avg_dist, r.max_reachable, r.avg_reachable}});
    log_worst_dist_vertex();

    m_converged = r.max_reachable <= tol;
    if (m_converged) {
        logger().info(
            "Converged ([max phi residual] {} <= {} [offset_residual_rel x target_distance])",
            r.max_reachable,
            tol);
    }

    if (!m_converged) {
        logger().warn(
            "Optimization did not converge ([max phi residual] {} > {} [offset_residual_rel x "
            "target_distance])",
            r.max_reachable,
            tol);

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
