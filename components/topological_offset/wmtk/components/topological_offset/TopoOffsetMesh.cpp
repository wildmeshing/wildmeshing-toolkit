
#include "TopoOffsetMesh.h"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <igl/predicates/predicates.h>
#include <igl/Timer.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <paraviewo/VTUWriter.hpp>

// #include <cmath>
// #include <limits>


namespace wmtk::components::topological_offset {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}


// assumes tag has been found. won't be called otherwise
void TopoOffsetMesh::init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXd& T_tags)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tags.rows());
    m_tags_count = T_tags.cols();

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(6 * T.rows());
    m_face_attribute.m_attributes.resize(4 * T.rows());
    m_tet_attribute.m_attributes.resize(T.rows());

    // propogate labels to tets
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        for (int i = 0; i < m_tags_count; i++) {
            m_tet_attribute[t_id].tags.push_back(T_tags(t_id, i));
        }

        if (m_params.offset_tags.size() == 1) { // 'single body' mode
            if (m_tet_attribute[t_id].tags[m_params.offset_tags[0][0]] ==
                m_params.offset_tags[0][1]) { // tet is in input body

                m_tet_attribute[t_id].label = 1;

                // propagate to children faces, edges, and verts
                auto v_ids = oriented_tet_vids(t_id);
                for (const size_t& v_id : v_ids) {
                    m_vertex_attribute[v_id].label = 1;
                }
                for (int i = 0; i < 6; i++) {
                    m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                }
                for (int i = 0; i < 4; i++) {
                    m_face_attribute[tuple_from_face(t_id, i).fid(*this)].label = 1;
                }
            }
        }
    }

    // set vertex positions
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        m_vertex_attribute[v_id].m_posf = V.row(v_id);
    }

    // if not 'single body mode', compute intersection region of offset_tags
    if (m_params.offset_tags.size() > 1) {
        auto faces = get_faces();
        for (const Tuple& f : faces) {
            if (face_in_tag_intersection(f)) {
                m_face_attribute[f.fid(*this)].label = 1;

                // propagate to children
                m_edge_attribute[f.eid(*this)].label = 1;
                m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
                m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                m_vertex_attribute[f.vid(*this)].label = 1;
                m_vertex_attribute[f.switch_vertex(*this).vid(*this)].label = 1;
                m_vertex_attribute[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        auto edges = get_edges();
        for (const Tuple& e : edges) {
            size_t e_id = e.eid(*this);
            if (m_edge_attribute[e_id].label != 1 && edge_in_tag_intersection(e)) {
                m_edge_attribute[e_id].label = 1;
                m_vertex_attribute[e.vid(*this)].label = 1;
                m_vertex_attribute[e.switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            if (m_vertex_attribute[v_id].label != 1 && vertex_in_tag_intersection(v)) {
                m_vertex_attribute[v_id].label = 1;
            }
        }
    }
}


void TopoOffsetMesh::init_input_complex_bvh()
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
            face_in_closure[simplex::Face(vs[2], vs[3], vs[1])] = true;
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
        if (!face_in_closure[f_simp] && m_face_attribute[f_id].label == 1) {
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
        if (!vertex_in_closure[v_id] && m_vertex_attribute[v_id].label == 1) {
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


bool TopoOffsetMesh::is_simplicially_embedded() const
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


bool TopoOffsetMesh::tet_is_simp_emb(const Tuple& t) const
{
    size_t t_id = t.tid(*this);
    if (m_tet_attribute[t_id].label != 0) { // entire tet in input/offset
        return true;
    }

    auto vs = oriented_tet_vids(t);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_attribute[vs[i]].label != 0) {
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
        return (m_face_attribute[glob_fid].label != 0);
    } else { // all four verts in complex but tet isn't, can't be simplicially embedded
        return false;
    }
}


void TopoOffsetMesh::simplicial_embedding()
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
                if (m_face_attribute[glob_fid].label == 0) {
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
        split_tet(t, garbage);
    }
    logger().info("\tTets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_attribute[f.fid(*this)].label == 0) {
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
        split_face(t, garbage);
    }
    logger().info("\tFaces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 0) { // edge not in input
            size_t v1_id = e.vid(*this);
            size_t v2_id = switch_vertex(e).vid(*this);
            if ((m_vertex_attribute[v1_id].label != 0) && (m_vertex_attribute[v2_id].label != 0)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        std::vector<Tuple> garbage;
        split_edge(t, garbage);
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}


void TopoOffsetMesh::marching_tets()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);

        // if one background and other in input/offset
        if ((m_vertex_attribute[v1].label == 0) != (m_vertex_attribute[v2].label == 0)) {
            e_to_split.emplace_back(v1, v2);
        }
    }

    // sort edges (split longest first), should give better output mesh quality
    if (m_params.sorted_marching) {
        logger().info("\tSorting edges by length...");
        sort_edges_by_length(e_to_split);
    }

    // actually split edges
    std::vector<Tuple> garbage;
    std::vector<size_t> frontier_verts; // one ring of these verts are in offset
    for (const simplex::Edge& e : e_to_split) {
        // determine which vertex in input/offset
        size_t v_in = e.vertices()[0];
        if (m_vertex_attribute[v_in].label == 0) {
            v_in = e.vertices()[1];
        }

        // split edge
        garbage.clear();
        Tuple t = tuple_from_edge(e.vertices());
        if (split_edge(t, garbage)) { // should never fail
            frontier_verts.push_back(v_in);
        } else {
            log_and_throw_error("edge split failed!");
        }
    }

    // mark all offset tets
    for (const size_t& v_id : frontier_verts) {
        auto t_ids = get_one_ring_tids_for_vertex(v_id);
        for (const size_t& t_id : t_ids) {
            if (m_tet_attribute[t_id].label == 0) {
                m_tet_attribute[t_id].label = 2;
            }
        }
    }
}


void TopoOffsetMesh::grow_offset_conservative()
{
    std::queue<Tuple> tets_q;
    auto all_tets = get_tets();

    for (const Tuple& t : all_tets) {
        if (tet_consistent_topology(t.tid(*this))) {
            tets_q.push(t);
        }
    }

    while (!tets_q.empty()) {
        Tuple curr_tet = tets_q.front();
        tets_q.pop();

        size_t tet_id = curr_tet.tid(*this);
        if (m_tet_attribute[tet_id].label != 0) { // already in offset
            continue;
        }

        // ensure tet wouldn't change topology
        if (!tet_consistent_topology(tet_id)) {
            continue;
        }

        bool in_offset = tet_is_in_offset_conservative(
            tet_id,
            m_params.relative_ball_threshold * m_params.target_distance);
        if (in_offset) {
            m_tet_attribute[tet_id].label = 2;
            for (int i = 0; i < 4; i++) { // propagate label to faces
                size_t f_id = tuple_from_face(tet_id, i).fid(*this);
                if (m_face_attribute[f_id].label != 1) {
                    m_face_attribute[f_id].label = 2;
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
                if (m_vertex_attribute[v_id].label != 1) {
                    m_vertex_attribute[v_id].label = 2;
                }
            }

            auto adj_tets = get_face_adjacent_tets(curr_tet);
            for (const Tuple& t : adj_tets) {
                if (m_tet_attribute[t.tid(*this)].label == 2) {
                    continue;
                }
                tets_q.push(t);
            }
        }
    }
}


void TopoOffsetMesh::set_offset_tet_tags()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 2) {
            for (const auto& tag : m_params.offset_tag_val) {
                if (tag[0] >= m_tet_attribute[t_id].tags.size()) {
                    log_and_throw_error(
                        "offset_tag_val [{}, {}] given, but tag_{} does not exist in tri mesh",
                        tag[0],
                        tag[1],
                        tag[0]);
                }
                m_tet_attribute[t_id].tags[tag[0]] = tag[1];
            }
        }
    }
}


bool TopoOffsetMesh::invariants(const std::vector<Tuple>& tets)
{
    igl::predicates::exactinit();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);

        if (res != igl::predicates::Orientation::NEGATIVE) {
            return false;
        }
    }
    return true;
}


void TopoOffsetMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(vertex_size(), -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<std::vector<int>> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset; // vertices to offset
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].label == 1) {
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
            cells.push_back(curr_e);
        }
    }

    // get all offset input faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        if (m_face_attribute[f.fid(*this)].label == 1) {
            std::vector<int> curr_f;
            curr_f.push_back(vid_map[f.vid(*this)]);
            Tuple f1 = switch_vertex(f);
            curr_f.push_back(vid_map[f1.vid(*this)]);
            Tuple f2 = switch_vertex(switch_edge(f1));
            curr_f.push_back(vid_map[f2.vid(*this)]);
            cells.push_back(curr_f);
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
            cells.push_back(curr_t);
        }
    }

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void TopoOffsetMesh::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);

    consolidate_mesh();
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);

    V.setZero();
    T.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));

    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);

        // set tet tags
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](t_id, 0) = m_tet_attribute[t_id].tags[i];
        }

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

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (int i = 0; i < m_tags_count; i++) {
        writer->add_cell_field(fmt::format("tag_{}", i), tags[i]);
    }

    writer->write_mesh(out_path, V, T);
}


void TopoOffsetMesh::write_msh(const std::string& file)
{
    logger().info("Write {}.msh", file);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    for (int j = 0; j < m_tags_count; j++) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", j), [&](size_t i) {
            return m_tet_attribute[i].tags[j];
        });
    }

    msh.save(file + ".msh", true);
}


} // namespace wmtk::components::topological_offset