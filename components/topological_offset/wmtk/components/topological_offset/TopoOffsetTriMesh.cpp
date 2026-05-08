#include "TopoOffsetTriMesh.h"
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/predicates/predicates.h>
#include <igl/remove_unreferenced.h>
#include <paraviewo/VTUWriter.hpp>
#include <queue>
#include <wmtk/utils/io.hpp>


namespace wmtk::components::topological_offset {


VertexAttributes2d::VertexAttributes2d(const Vector2d& p)
    : m_posf(p)
{}


void TopoOffsetTriMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& F,
    const MatrixSi& F_tags,
    const MatrixXd& V_env,
    const MatrixXi& F_env,
    const std::vector<std::string>& tag_names)
{
    // assert dimensions
    assert(V.cols() == 2);
    assert(F.cols() == 3);
    assert(F.rows() == F_tags.rows());
    assert((V_env.rows() == 0) || (V_env.cols() == 2));
    assert((F_env.rows() == 0) || (F_env.cols() == 2)); // edges, envelope
    m_tags_count = F_tags.cols();

    // initialize connectivity
    init(F);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(3 * F.rows());
    m_face_attribute.m_attributes.resize(F.rows());

    // set envelope data
    if (V_env.rows() > 0) {
        logger().info(
            "Envelope ({} vertices, {} edges) found. will be retained in output.",
            V_env.rows(),
            F_env.rows());
        m_has_envelope = true;
        m_V_envelope = V_env;
        m_F_envelope = F_env;
    }

    // check that all tags are present
    for (const std::set<std::string>& nameset : m_params.offset_tags) {
        for (const std::string& name : nameset) {
            if (std::find(tag_names.begin(), tag_names.end(), name) == tag_names.end()) {
                log_and_throw_error("Given tag '{}' in offset_tags does not exist in mesh.", name);
            }
        }
    }

    // set tag string/id maps
    for (int64_t i = 0; i < tag_names.size(); i++) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }
    for (const std::string& tag : m_params.offset_output_tag) {
        if (std::find(tag_names.begin(), tag_names.end(), tag) == tag_names.end()) {
            int64_t new_id = m_tag_id_to_name.size();
            m_tag_id_to_name[new_id] = tag;
            m_tag_name_to_id[tag] = new_id;
            m_tags_count++;
        }
    }
    for (const std::set<std::string>& nameset : m_params.offset_tags) {
        std::set<int64_t> idset;
        for (const std::string& name : nameset) {
            idset.insert(m_tag_name_to_id[name]);
        }
        m_offset_tags_ids.push_back(idset);
    }
    for (const std::string& name : m_params.offset_output_tag) {
        m_offset_output_tag_ids.insert(m_tag_name_to_id[name]);
    }

    // propagate tags to faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        for (int j = 0; j < F_tags.cols(); j++) {
            if (F_tags.coeff(f_id, j) == 1) {
                m_face_attribute[f_id].tag.insert(j);
            }
        }
    }

    // set position of verts
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        m_vertex_attribute[v_id].m_posf = V.row(v_id);
    }

    // compute intersection of unions for faces.
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        bool in_input = true; // json specifies at least one tag set, so can safely set this.
        const std::set<int64_t> f_tag = m_face_attribute[f_id].tag;
        for (const std::set<int64_t>& tag : m_offset_tags_ids) {
            if (!any_tag_present(f_tag, tag)) {
                in_input = false;
                break;
            }
        }
        if (in_input) {
            m_face_attribute[f_id].label = 1;
            // propagate to edges and verts in tri
            m_edge_attribute[f.eid(*this)].label = 1;
            m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
            m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
            m_vertex_attribute[f.vid(*this)].label = 1;
            m_vertex_attribute[f.switch_vertex(*this).vid(*this)].label = 1;
            m_vertex_attribute[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
        }
    }

    // compute intersection of unions for edges.
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        // check if edge already in input via face
        size_t e_id = e.eid(*this);
        if (m_edge_attribute[e_id].label == 1) {
            continue;
        }

        // gather incident face(s)
        std::vector<std::set<int64_t>> inc_tags;
        inc_tags.push_back(m_face_attribute[e.fid(*this)].tag);
        auto other = e.switch_face(*this);
        if (other) {
            inc_tags.push_back(m_face_attribute[other.value().fid(*this)].tag);
        }

        // check if criteria met
        bool in_input = true;
        for (const std::set<int64_t>& tag : m_offset_tags_ids) {
            bool union_present = false;
            for (const std::set<int64_t>& inc_tag : inc_tags) {
                if (any_tag_present(inc_tag, tag)) {
                    union_present = true;
                }
            }
            if (!union_present) {
                in_input = false;
                break;
            }
        }
        if (in_input) {
            m_edge_attribute[e_id].label = 1;
            // propagate to vertices
            m_vertex_attribute[e.vid(*this)].label = 1;
            m_vertex_attribute[e.switch_vertex(*this).vid(*this)].label = 1;
        }
    }

    // compute intersection of unions for vertices.
    for (const Tuple& v : verts) {
        // check if vertex already in input via face or edge
        size_t v_id = v.vid(*this);
        if (m_vertex_attribute[v_id].label == 1) {
            continue;
        }

        // get incident face tags
        auto inc_fids = get_one_ring_fids_for_vertex(v_id);
        std::vector<std::set<int64_t>> inc_tags;
        for (const size_t& f_id : inc_fids) {
            inc_tags.push_back(m_face_attribute[f_id].tag);
        }

        // check if vertex meets criteria
        bool in_input = true;
        for (const std::set<int64_t>& tag : m_offset_tags_ids) {
            bool union_present = false;
            for (const std::set<int64_t>& inc_tag : inc_tags) {
                if (any_tag_present(inc_tag, tag)) {
                    union_present = true;
                }
            }
            if (!union_present) {
                in_input = false;
                break;
            }
        }
        if (in_input) {
            m_vertex_attribute[v_id].label = 1;
        }
    }
}


bool TopoOffsetTriMesh::empty_input_complex()
{
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (m_vertex_attribute[v_id].label == 1) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTriMesh::init_input_complex_bvh()
{
    // used a few times. just collect once
    auto faces = get_faces();
    auto edges = get_edges();
    auto verts = get_vertices();

    // to check if an edge is in closure of input complex faces
    std::map<simplex::Edge, bool> edge_in_closure;
    for (const Tuple& e : edges) {
        edge_in_closure[simplex_from_edge(e)] = false;
    }

    // to check if vertex is in closure of input complex faces and edges
    std::map<size_t, bool> vertex_in_closure;
    for (const Tuple& v : verts) {
        vertex_in_closure[v.vid(*this)] = false;
    }

    // collect faces in input complex
    std::vector<simplex::Face> complex_faces;
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].label == 1) {
            size_t v0 = f.vid(*this);
            size_t v1 = f.switch_vertex(*this).vid(*this);
            size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
            complex_faces.emplace_back(v0, v1, v2);
            edge_in_closure[simplex::Edge(v0, v1)] = true;
            edge_in_closure[simplex::Edge(v1, v2)] = true;
            edge_in_closure[simplex::Edge(v0, v2)] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
            vertex_in_closure[v2] = true;
        }
    }

    // collect edges in input complex that are not contained in a face
    std::vector<simplex::Edge> complex_edges;
    for (const Tuple& e : edges) {
        simplex::Edge e_simp = simplex_from_edge(e);
        if (!edge_in_closure[e_simp] && m_edge_attribute[e.eid(*this)].label == 1) {
            size_t v0 = e.vid(*this);
            size_t v1 = e.switch_vertex(*this).vid(*this);
            complex_edges.emplace_back(v0, v1);
            edge_in_closure[e_simp] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
        }
    }

    // collect vertices in input complex not contained in an edge or face
    std::vector<size_t> complex_verts;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (!vertex_in_closure[v_id] && m_vertex_attribute[v_id].label == 1) {
            complex_verts.push_back(v_id);
            vertex_in_closure[v_id] = true;
        }
    }

    // extract vertices included in simplicial complex
    std::vector<Vector2d> V_vec;
    std::map<size_t, size_t> v_index_map; // new = map[old]
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (vertex_in_closure[v_id]) {
            v_index_map[v_id] = V_vec.size();
            V_vec.push_back(m_vertex_attribute[v_id].m_posf);
        }
    }
    MatrixXd V(V_vec.size(), 2);
    for (int i = 0; i < V_vec.size(); i++) {
        V.row(i) = V_vec[i];
    }

    MatrixXi T(0, 4); // no tets

    MatrixXi F(complex_faces.size(), 3); // faces
    int index = 0;
    for (const simplex::Face& f_simp : complex_faces) {
        auto vs = f_simp.vertices();
        // NOTE: does id order matter here (i.e., in BVH class?)
        F.row(index) << v_index_map[vs[0]], v_index_map[vs[1]], v_index_map[vs[2]];
        index++;
    }

    MatrixXi E(complex_edges.size(), 2); // isolated edges
    index = 0;
    for (const simplex::Edge& e_simp : complex_edges) {
        auto vs = e_simp.vertices();
        E.row(index) << v_index_map[vs[0]], v_index_map[vs[1]];
        index++;
    }

    MatrixXi P(complex_verts.size(), 1); // isolated vertices
    index = 0;
    for (const size_t& v_id : complex_verts) {
        P(index, 0) = v_index_map[v_id];
        index++;
    }

    // set BVH
    m_input_complex_bvh.clear(); // in case resetting it now
    m_input_complex_bvh.init(V, T, F, E, P);
}


bool TopoOffsetTriMesh::is_simplicially_embedded() const
{
    int bad_tris = 0;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        bad_tris += (!tri_is_simp_emb(f));
    }
    if (bad_tris == 0) {
        logger().info("\tInput complex/offset simplicially embedded: TRUE");
        return true;
    } else {
        logger().info(
            "\tInput complex/offset simplicially embedded: FALSE ({} bad tris)",
            bad_tris);
        return false;
    }
}


bool TopoOffsetTriMesh::tri_is_simp_emb(const Tuple& t) const
{
    size_t f_id = t.fid(*this);
    if (m_face_attribute[f_id].label != 0) { // entire tri in input
        return true;
    }

    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 3; i++) {
        if (m_vertex_attribute[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }

    if (vs_in.size() <= 1) { // nothing or just one vert
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs_in[0], vs_in[1]));
        return (m_edge_attribute[e_id].label != 0);
    } else { // all 3 verts in input but tri isnt, cant be simplicially embedded
        return false;
    }
}


void TopoOffsetTriMesh::simplicial_embedding()
{
    // identify tris to split
    std::vector<simplex::Face> tris_to_split;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        size_t f_id = f.fid(*this);
        auto vs = oriented_tri_vids(f_id);
        if (m_face_attribute[f_id].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t e_id = edge_id_from_simplex(simplex::Edge(v1, v2));
                if (m_edge_attribute[e_id].label == 0) {
                    to_split = false;
                    break;
                }
            }

            if (to_split) { // tri not in input but all edges are
                tris_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split tris
    for (const simplex::Face& f : tris_to_split) {
        const auto& vs = f.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2]);
        std::vector<Tuple> garbage;
        if (!split_face(t, garbage)) {
            log_and_throw_error("face split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tTris split: {}", tris_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t e_id = e.eid(*this);
        if (m_edge_attribute[e_id].label == 0) {
            size_t v1_id = e.vid(*this);
            size_t v2_id = e.switch_vertex(*this).vid(*this);
            if ((m_vertex_attribute[v1_id].label != 0) && (m_vertex_attribute[v2_id].label != 0)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = get_tuple_from_edge(e);
        std::vector<Tuple> garbage;
        if (!split_edge(t, garbage)) {
            log_and_throw_error("edge split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}


void TopoOffsetTriMesh::marching_tets()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);

        // if one background and the other input/offset
        if ((m_vertex_attribute[v1].label == 0) != (m_vertex_attribute[v2].label == 0)) {
            e_to_split.emplace_back(v1, v2);
        }
    }

    // sort edges by length
    if (m_params.sorted_marching) {
        logger().info("\tSorting edges by length...");
        sort_edges_by_length(e_to_split);
    }

    // actually split edges
    std::vector<Tuple> garbage;
    std::vector<size_t> frontier_verts; // the one-ring of these verts must be labelled offset
    for (const simplex::Edge& e : e_to_split) {
        // get vert of edge in offset
        size_t v_in = e.vertices()[0];
        if (m_vertex_attribute[v_in].label == 0) {
            v_in = e.vertices()[1];
        }

        // split edge
        garbage.clear();
        Tuple t = get_tuple_from_edge(e);
        if (split_edge(t, garbage)) { // this should never fail
            frontier_verts.push_back(v_in);
        } else {
            log_and_throw_error("edge split failed! (marching_tets)");
        }
    }

    // mark all offset tris (incident to any vert with label 1 or 2)
    for (const size_t v_id : frontier_verts) {
        auto tris = get_one_ring_tris_for_vertex(tuple_from_vertex(v_id));
        for (const Tuple& t : tris) {
            size_t f_id = t.fid(*this);
            if (m_face_attribute[f_id].label == 0) { // dont want to overwrite if in input
                m_face_attribute[f_id].label = 2;
                m_face_attribute[f_id].tag = TEMP_OFFSET_TRI_TAG;
                // propagate to children
                auto vs = oriented_tri_vids(f_id);
                for (int i = 0; i < 3; i++) {
                    if (m_vertex_attribute[vs[i]].label != 1) {
                        m_vertex_attribute[vs[i]].label = 2;
                    }
                    size_t e_id = tuple_from_edge(f_id, i).eid(*this);
                    if (m_edge_attribute[e_id].label != 1) {
                        m_edge_attribute[e_id].label = 2;
                    }
                }
            }
        }
    }
}


void TopoOffsetTriMesh::grow_offset_conservative()
{
    std::queue<Tuple> tris_q;
    auto all_tris = get_faces();

    for (const Tuple& f : all_tris) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].label == 0 && offset_tri_consistent_topology(f_id)) {
            tris_q.push(f);
        }
    }
    logger().info("\tInitial queue size {}", tris_q.size());

    while (!tris_q.empty()) {
        Tuple curr_tri = tris_q.front();
        tris_q.pop();

        size_t tri_id = curr_tri.fid(*this);
        if (m_face_attribute[tri_id].label != 0) { // already in offset
            continue;
        }

        // ensure tri doesn't change topology
        if (!offset_tri_consistent_topology(tri_id)) {
            continue;
        }

        // if tri is within offset, include in front and for all edge adjacent tris, add to queue if
        // adjacent to front via 2 or fewer edges
        bool in_offset = tri_is_in_offset_conservative(
            tri_id,
            m_params.relative_ball_threshold * m_params.target_distance);
        if (in_offset) {
            m_face_attribute[tri_id].label = 2;
            m_face_attribute[tri_id].tag = TEMP_OFFSET_TRI_TAG;
            auto vs = oriented_tri_vids(tri_id);
            for (int i = 0; i < 3; i++) { // propagate labels to edges and verts
                if (m_vertex_attribute[vs[i]].label != 1) {
                    m_vertex_attribute[vs[i]].label = 2;
                }
                size_t e_id = tuple_from_edge(tri_id, i).eid(*this);
                if (m_edge_attribute[e_id].label != 1) {
                    m_edge_attribute[e_id].label = 2;
                }
            }

            // collect edge adjacent faces, add to queue if they dont change topology
            auto adj_tris = get_edge_adjacent_faces(curr_tri);
            for (const Tuple& f : adj_tris) {
                if (m_face_attribute[f.fid(*this)].label == 2) {
                    continue;
                }
                tris_q.push(f);
            }
        }
    }
}


void TopoOffsetTriMesh::set_offset_tri_tags()
{
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].label == 2) {
            m_face_attribute[f_id].tag = m_offset_output_tag_ids;
        }
    }
}


bool TopoOffsetTriMesh::offset_is_manifold()
{
    // vertex map
    std::map<size_t, bool> included_vids;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        included_vids[v.vid(*this)] = false;
    }

    // collect faces in closed offset region (labelled 1 or 2)
    auto tris = get_faces();
    std::vector<Vector3i> offset_tris;
    for (const Tuple& t : tris) {
        size_t t_id = t.fid(*this);
        if (m_face_attribute[t_id].label != 0) {
            auto vs = oriented_tri_vids(t_id);
            offset_tris.emplace_back(vs[0], vs[1], vs[2]);
            included_vids[vs[0]] = true;
            included_vids[vs[1]] = true;
            included_vids[vs[2]] = true;
        }
    }

    // create consolidated v_id map
    int vert_count = 0;
    std::map<size_t, size_t> v_id_map;
    for (const auto& pair : included_vids) {
        if (pair.second) {
            v_id_map[pair.first] = vert_count++;
        }
    }

    // form matrix
    MatrixXi F(offset_tris.size(), 3);
    for (int i = 0; i < offset_tris.size(); i++) {
        for (int j = 0; j < 3; j++) {
            F(i, j) = v_id_map[offset_tris[i](j)];
        }
    }

    // check manifoldness
    bool is_edge_man = igl::is_edge_manifold(F);
    VectorXi B;
    bool is_vert_man = igl::is_vertex_manifold(F, B);
    return (is_edge_man && is_vert_man);
}


bool TopoOffsetTriMesh::invariants(const std::vector<Tuple>& tris)
{
    igl::predicates::exactinit();
    for (const Tuple& t : tris) {
        auto vs = oriented_tri_vids(t);

        auto res = igl::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res != igl::predicates::Orientation::POSITIVE) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTriMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(
        get_vertices().size(),
        -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<std::vector<int>> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].label == 1) {
            Eigen::Vector2d p = m_vertex_attribute[i].m_posf;
            verts_to_offset.emplace_back(p(0), p(1), 0.0);
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
            curr_e.push_back(vid_map[e.switch_vertex(*this).vid(*this)]);
            cells.push_back(curr_e);
        }
    }

    // get all offset input triangles
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].label == 1) {
            auto v_ids = oriented_tri_vids(f_id);
            std::vector<int> curr_f;
            for (const size_t v_id : v_ids) {
                curr_f.push_back(vid_map[v_id]);
            }
            cells.push_back(curr_f);
        }
    }

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void TopoOffsetTriMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    consolidate_mesh();
    const auto& vs = get_vertices();
    const auto& tris = get_faces();

    bool has_tmp_tag = false;
    for (const Tuple& f : tris) {
        if (m_face_attribute[f.fid(*this)].tag == TEMP_OFFSET_TRI_TAG) {
            has_tmp_tag = true;
            continue;
        }
    }

    Eigen::MatrixXd V(vs.size(), 2);
    Eigen::MatrixXi F(tris.size(), 3);

    V.setZero();
    F.setZero();

    // hacky (has_tmp_tag)
    std::vector<MatrixXd> tags(m_tags_count + (has_tmp_tag ? 1 : 0), MatrixXd(tris.size(), 1));

    for (const Tuple& f : tris) {
        size_t f_id = f.fid(*this);

        // also hacky but whatever.
        if (has_tmp_tag && (m_face_attribute[f_id].tag == TEMP_OFFSET_TRI_TAG)) {
            tags[m_tags_count](f_id, 0) = 1;
            for (int j = 0; j < m_tags_count; j++) {
                tags[j](f_id, 0) = 0;
            }
            continue;
        }

        // set tri tags
        for (int j = 0; j < m_tags_count; j++) {
            tags[j](f_id, 0) = (m_face_attribute[f_id].tag.count(j) == 1) ? 1 : 0;
        }
    }

    for (const Tuple& f : tris) {
        // set tri verts
        const auto& loc_vs = oriented_tri_vertices(f);
        for (int j = 0; j < 3; j++) {
            F(f.fid(*this), j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t v_id = v.vid(*this);
        V.row(v_id) = m_vertex_attribute[v_id].m_posf;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    for (int64_t i = 0; i < m_tags_count; i++) {
        writer->add_cell_field(m_tag_id_to_name[i], tags[i]);
    }
    if (has_tmp_tag) {
        writer->add_cell_field("offset_tag", tags[m_tags_count]); // also hacky but it works.
    }
    writer->write_mesh(path + ".vtu", V, F);

    // surface output
    if (m_has_envelope) {
        const auto out_surf_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        logger().info("Write {}", out_surf_path);
        surf_writer->write_mesh(out_surf_path, m_V_envelope, m_F_envelope);
    }
}


void TopoOffsetTriMesh::write_msh(const std::string& file)
{
    logger().info("Write {}.msh", file);
    consolidate_mesh();

    wmtk::MshData msh;

    // set vertices
    const auto& verts = get_vertices();
    msh.add_face_vertices(verts.size(), [&](size_t k) {
        size_t i = verts[k].vid(*this);
        Vector2d p2d = m_vertex_attribute[i].m_posf;
        return Vector3d(p2d(0), p2d(1), 0.0);
    });

    // set faces
    const auto& tris = get_faces();
    msh.add_faces(tris.size(), [&](size_t k) {
        auto i = tris[k].fid(*this);
        auto vs = oriented_tri_vids(i);
        std::array<size_t, 3> f_verts;
        for (int j = 0; j < 3; j++) {
            f_verts[j] = vs[j];
            assert(f_verts[j] < verts.size());
        }
        return f_verts;
    });

    // add tags under ImageVolume physical group
    for (int64_t j = 0; j < m_tags_count; j++) {
        msh.add_face_attribute<1>(m_tag_id_to_name[j], [&](size_t i) {
            return ((m_face_attribute[i].tag.count(j) == 1) ? 1 : 0);
        });
    }
    msh.add_physical_group("ImageVolume");

    if (m_has_envelope) {
        msh.add_edge_vertices(m_V_envelope.rows(), [this](size_t k) {
            return Vector3d(m_V_envelope(k, 0), m_V_envelope(k, 1), 0);
        });
        msh.add_edges(m_F_envelope.rows(), [this](size_t k) { return m_F_envelope.row(k); });
        msh.add_physical_group("EnvelopeSurface");
    }

    msh.save(file + ".msh", true);
}


void TopoOffsetTriMesh::write_msh_groups(const std::string& file)
{
    logger().info("Write {}.msh", file);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& faces = get_faces();

    std::vector<Tuple> faces_with_tag;
    faces_with_tag.reserve(faces.size());

    auto msh_add_faces = [&]() {
        msh.add_faces(faces_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tri_vids(faces_with_tag[k]);
            std::array<size_t, 3> data;
            for (int j = 0; j < 3; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // group for each tag
    for (int64_t tag_img = 0; tag_img < m_tags_count; tag_img++) {
        faces_with_tag.clear();
        for (const Tuple& f : faces) {
            size_t f_id = f.fid(*this);
            if (m_face_attribute[f_id].tag.count(tag_img) != 0) {
                faces_with_tag.push_back(f);
            }
        }

        if (faces_with_tag.empty()) {
            continue;
        }

        if (tag_img == 0) {
            // set vertices
            const auto& verts = get_vertices();
            msh.add_face_vertices(verts.size(), [&](size_t k) {
                auto i = verts[k].vid(*this);
                Vector2d p2 = m_vertex_attribute[i].m_posf;
                return Vector3d(p2(0), p2(1), 0);
            });
        } else {
            msh.add_empty_vertices(2);
        }
        msh_add_faces();

        const std::string group_name = m_tag_id_to_name[tag_img];
        msh.add_physical_group(group_name);
    }

    if (m_has_envelope) {
        msh.add_edge_vertices(m_V_envelope.rows(), [this](size_t k) {
            return Vector3d(m_V_envelope(k, 0), m_V_envelope(k, 1), 0);
        });
        msh.add_edges(m_F_envelope.rows(), [this](size_t k) { return m_F_envelope.row(k); });
        msh.add_physical_group("EnvelopeSurface");
    }

    msh.save(file + ".msh", true);
}


} // namespace wmtk::components::topological_offset
