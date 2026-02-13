#include "TopoOffsetTriMesh.h"
#include <igl/predicates/predicates.h>
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
    const MatrixXd& F_tags,
    const std::vector<std::string>& all_tag_names)
{
    // assert dimensions
    assert(V.cols() == 2);
    assert(F.cols() == 3);
    assert(F.rows() == F_tags.rows());

    // save tags info to mesh (assumes desired tag is in tag name list)
    auto it = std::find(
        all_tag_names.begin(),
        all_tag_names.end(),
        m_params.tag_name); // note: this is a const iterator
    m_toi_ind = std::distance(all_tag_names.begin(), it);
    m_tags_count = all_tag_names.size();
    for (const auto& name : all_tag_names) {
        m_all_tag_names.push_back(name);
    }

    // extract tag of interest
    MatrixXi F_tag = F_tags.col(m_toi_ind).cast<int>(); // toi vals as ints

    // initialize connectivity
    init(F);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(3 * F.rows());
    m_face_attribute.m_attributes.resize(F.rows());

    // propagate labels to faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        for (int i = 0; i < m_tags_count; i++) {
            m_face_attribute[f_id].tags.push_back(F_tags(f_id, i));
        }
    }

    // label boundary edges
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        // find unique incident face tags
        std::set<int> nb_tags;
        nb_tags.insert(F_tag(e.fid(*this), 0));
        auto other = e.switch_face(*this);
        if (other) {
            nb_tags.insert(F_tag(other.value().fid(*this), 0));
        }

        // count number of unique incident face tags in sep_tags
        if (count_sep_tags(nb_tags) >= 2) { // can be max of two (assume 2d edge manifold)
            size_t e_id = e.eid(*this);
            m_edge_attribute[e_id].label = 1;

            // label children vertices as input
            m_vertex_attribute[e.vid(*this)].label = 1;
            m_vertex_attribute[e.switch_vertex(*this).vid(*this)].label = 1;
        }
    }

    // label boundary vertices ( and set position)
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        // set coords
        size_t v_id = v.vid(*this);
        m_vertex_attribute[v_id].m_posf = V.row(v_id);

        // check if vertex already labeled from edge
        if (m_vertex_attribute[v_id].label == 1) {
            continue;
        }

        // collect incident face tags
        auto inc_f_ids = get_one_ring_tris_for_vertex(v);
        std::set<int> nb_tags;
        for (const Tuple& f : inc_f_ids) {
            nb_tags.insert(F_tag(f.fid(*this), 0));
        }

        // determine if on boundary of sep tags
        if (count_sep_tags(nb_tags) >= 2) {
            m_vertex_attribute[v_id].label = 1;
        }
    }
}


void TopoOffsetTriMesh::init_input_complex_bvh()
{
    m_input_complex_bvh.clear(); // in case resetting it now

    // extract edges and isolated verts in input complex
    std::vector<std::array<size_t, 2>> components;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            components.push_back({e.vid(*this), e.switch_vertex(*this).vid(*this)});
        }
    }

    auto verts = get_vertices();
    MatrixXd V(verts.size(), 2);
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        V.row(v_id) = m_vertex_attribute[v_id].m_posf;

        if (m_vertex_attribute[v_id].label != 1) {
            continue;
        }
        auto inc_edges = get_one_ring_edges_for_vertex(v);
        bool isolated = true;
        for (const Tuple& e : inc_edges) {
            if (m_edge_attribute[e.eid(*this)].label == 1) {
                isolated = false;
                break;
            }
        }
        if (isolated) components.push_back({v_id, v_id});
    }

    MatrixXi E(components.size(), 2); // copy edges (and isolated vert 'psuedoedges')
    int index = 0;
    for (const auto& comp : components) {
        E(index, 0) = comp[0];
        E(index, 1) = comp[1];
        index++;
    }

    m_input_complex_bvh.init(V, E, 1e-6);
}


bool TopoOffsetTriMesh::is_simplicially_embedded() const
{
    int bad_tris = 0;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        if (m_face_attribute[f.fid(*this)].label != 0) continue;
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
    if (m_face_attribute[f_id].label != 0) {
        logger().warn("Non-background tri tested for simp emb");
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
    } else { // all 3 verts in input, cant be simplicially embedded
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
        split_face(t, garbage);
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
        split_edge(t, garbage);
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
        }
    }

    // mark all offset tris (incident to any vert with label 1 or 2)
    for (const size_t v_id : frontier_verts) {
        auto tris = get_one_ring_tris_for_vertex(tuple_from_vertex(v_id));
        for (const Tuple& t : tris) {
            m_face_attribute[t.fid(*this)].label = 2;
        }
    }
}


void TopoOffsetTriMesh::grow_offset_conservative()
{
    std::queue<Tuple> tris_q;
    auto all_tris = get_faces();
    // std::map<size_t, bool> visited;

    for (const Tuple& f : all_tris) {
        // visited[f.fid(*this)] = false; // initialize visited array
        if (tri_consistent_topology(f.fid(*this))) {
            tris_q.push(f);
        }
    }

    int testing = 0;
    while (!tris_q.empty()) {
        testing++;
        if (testing % 10000 == 0) {
            logger().info("queue size: {}", tris_q.size());
        }

        Tuple curr_tri = tris_q.front();
        tris_q.pop();

        size_t tri_id = curr_tri.fid(*this);
        if (m_face_attribute[tri_id].label == 2) { // already in offset
            continue;
        }

        // ensure tri doesn't change topology
        if (!tri_consistent_topology(tri_id)) {
            continue;
        }

        // if tri is within offset, include in front and for all edge adjacent tris, add to queue if
        // adjacent to front via 2 or fewer edges
        bool in_offset = tri_is_in_offset_conservative(
            tri_id,
            m_params.relative_ball_threshold * m_params.target_distance);
        if (in_offset) {
            m_face_attribute[tri_id].label = 2;
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
            m_face_attribute[f_id].tags[m_toi_ind] = m_params.offset_tag_val;
        }
    }
}


bool TopoOffsetTriMesh::invariants(const std::vector<Tuple>& tets)
{
    igl::predicates::exactinit();
    for (const Tuple& t : tets) {
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

    // get all offset input triangles (should never happen)
    bool flag = false;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].label == 1) {
            flag = true;
            auto v_ids = oriented_tri_vids(f_id);
            std::vector<int> curr_f;
            for (const size_t v_id : v_ids) {
                curr_f.push_back(vid_map[v_id]);
            }
            cells.push_back(curr_f);
        }
    }
    if (flag) {
        logger().warn("One or more triangle included in complex to offset.");
    }

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void TopoOffsetTriMesh::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);

    consolidate_mesh();
    const auto& vs = get_vertices();
    const auto& tris = get_faces();

    Eigen::MatrixXd V(vs.size(), 3);
    Eigen::MatrixXi F(tris.size(), 3);

    V.setZero();
    F.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tris.size(), 1));

    for (const Tuple& f : tris) {
        size_t f_id = f.fid(*this);

        // set tet tags
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](f_id, 0) = m_face_attribute[f_id].tags[i];
        }

        // set tet verts
        const auto& loc_vs = oriented_tri_vertices(f);
        for (int j = 0; j < 3; j++) {
            F(f.fid(*this), j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t v_id = v.vid(*this);
        Vector2d p2d = m_vertex_attribute[v_id].m_posf;
        V.row(v_id) = Vector3d(p2d(0), p2d(1), 0.0);
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    int index = 0;
    for (const std::string& tag_name : m_all_tag_names) {
        writer->add_cell_field(tag_name, tags[index]);
        index++;
    }

    writer->write_mesh(out_path, V, F);
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

    // set tags perface
    for (size_t j = 0; j < m_tags_count; j++) {
        msh.add_face_attribute<1>(m_all_tag_names[j], [&](size_t i) {
            return m_face_attribute[i].tags[j];
        });
    }

    msh.save(file + ".msh", true);
}

} // namespace wmtk::components::topological_offset