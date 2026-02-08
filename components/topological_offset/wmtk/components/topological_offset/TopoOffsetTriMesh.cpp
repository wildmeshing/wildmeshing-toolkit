#include "TopoOffsetTriMesh.h"
#include <igl/predicates/predicates.h>
#include <paraviewo/VTUWriter.hpp>


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


bool TopoOffsetTriMesh::is_simplicially_embedded() const
{
    int bad_tris = 0;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        bad_tris += (!tri_is_simp_emb(f));
    }
    if (bad_tris == 0) {
        logger().info("\tBoundary simplicially embedded: TRUE");
        return true;
    } else {
        logger().info("\tBoundary simplicially embedded: FALSE ({} bad tris)", bad_tris);
        return false;
    }
}


bool TopoOffsetTriMesh::tri_is_simp_emb(const Tuple& t) const
{
    size_t f_id = t.fid(*this);
    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 3; i++) {
        if (m_vertex_attribute[vs[i]].label == 1) {
            vs_in.push_back(vs[i]);
        }
    }

    if (vs_in.size() <= 1) { // nothing or just one vert
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs_in[0], vs_in[1]));
        return (m_edge_attribute[e_id].label == 1);
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
        if (m_face_attribute[f_id].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t e_id = edge_id_from_simplex(simplex::Edge(v1, v2));
                if (m_edge_attribute[e_id].label != 1) {
                    to_split = false;
                    break;
                }
            }

            if (to_split) { // tri not in input but all edges are
                tris_to_split.push_back(simplex::Face());
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
        if (m_edge_attribute[e_id].label != 1) {
            size_t v1_id = e.vid(*this);
            size_t v2_id = e.switch_vertex(*this).vid(*this);
            if ((m_vertex_attribute[v1_id].label == 1) && (m_vertex_attribute[v2_id].label == 1)) {
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


void TopoOffsetTriMesh::perform_offset()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        if ((m_vertex_attribute[v1].label + m_vertex_attribute[v2].label) == 1) {
            e_to_split.push_back(simplex::Edge(v1, v2));
        }
    }

    // sort edges by length
    logger().info("\tSorting edges by length...");
    sort_edges_by_length(e_to_split);

    // actually split edges
    std::vector<Tuple> new_edges;
    for (const simplex::Edge& e : e_to_split) {
        // split edge
        new_edges.clear();
        Tuple t = get_tuple_from_edge(e);
        split_edge(t, new_edges);
    }

    // mark all offset tris (incident to any vert with label 1)
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        size_t f_id = f.fid(*this);
        auto vs = oriented_tri_vids(f_id);
        bool in_offset = false;
        for (size_t v_id : vs) {
            if (m_vertex_attribute[v_id].label == 1) {
                in_offset = true;
                break;
            }
        }

        if (in_offset) {
            m_face_attribute[f_id].label = 2;
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


} // namespace wmtk::components::topological_offset