#pragma once
#include <igl/write_triangle_mesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>


namespace Edge2d {

class EdgeCollapse : public wmtk::TriMesh
{
public:
    std::vector<Eigen::Vector3d> m_vertex_positions;

    EdgeCollapse(std::vector<Eigen::Vector3d> _m_vertex_positions)
        : m_vertex_positions(_m_vertex_positions)
    {}

    ~EdgeCollapse() {}

    void compact_vertex_positions(std::vector<size_t> vid_newvid_map)
    {
        std::vector<Eigen::Vector3d> new_m_vertex_positions(m_vertex_positions.size());
        size_t cnt = 0;
        for (int i = 0; i < m_vertex_positions.size(); i++) {
            if (vid_newvid_map[i] == UINT64_MAX) continue;
            new_m_vertex_positions[cnt] = m_vertex_positions[i];
            cnt++;
        }
        new_m_vertex_positions.resize(cnt);
        m_vertex_positions = new_m_vertex_positions;
        // std::cout << m_vertex_positions.size() << std::endl;
    };

    // write the collapsed mesh into a obj
    void write_triangle_mesh(std::string path)
    {
        std::vector<size_t> vid_newvid_map = compact();
        compact_vertex_positions(vid_newvid_map);
        std::vector<VertexConnectivity> new_m_vertex_connectivity = get_m_vertex_connectivity();
        std::vector<TriangleConnectivity> new_m_tri_connectivity = get_m_tri_connectivity();

        Eigen::MatrixXd V(n_vertices(), 3);
        Eigen::MatrixXi F(n_triangles(), 3);

        for (int i = 0; i < n_vertices(); i++) V.row(i) = m_vertex_positions[i];


        for (int i = 0; i < n_triangles(); i++)
            F.row(i) = Eigen::Vector3i(
                (int)new_m_tri_connectivity[i].m_indices[0],
                (int)new_m_tri_connectivity[i].m_indices[1],
                (int)new_m_tri_connectivity[i].m_indices[2]);

        bool ok = igl::write_triangle_mesh(path, V, F);
        // std::cout << "write output is ok ? " << ok << std::endl;
    }


    bool collapse_shortest(int target_vertex_count);

    bool collapse_qec();
    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh::Tuple& t);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh::Tuple& t);

    double compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple);

    void update_position(size_t v1, size_t v2, Tuple& new_vert);

    void resize_attributes(size_t v, size_t t) 
    {
        m_vertex_positions.resize(v);
    }
};

class ElementInQueue
{
public:
    wmtk::TriMesh::Tuple edge;
    double weight;

    ElementInQueue() {}
    ElementInQueue(const wmtk::TriMesh::Tuple& e, double w)
        : edge(e)
        , weight(w)
    {}
};
struct cmp_l
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.get_vid() > e2.edge.get_vid();
        return e1.weight < e2.weight;
    }
};
struct cmp_s
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.get_vid() < e2.edge.get_vid();
        return e1.weight > e2.weight;
    }
};

} // namespace Edge2d
