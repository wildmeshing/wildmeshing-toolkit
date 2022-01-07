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

    struct CollapseInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    } collapse_cache;


    bool collapse_before(const Tuple& t) override;
    bool collapse_after(const Tuple& t) override;

    // write the collapsed mesh into a obj
    bool write_triangle_mesh(std::string path)
    {
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(m_vertex_positions.size(), 3);
        for (auto& t : get_vertices()) {
            auto i = t.vid(*this);
            V.row(i) = m_vertex_positions[i];
        }

        Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
        for (auto& t : get_faces()) {
            auto i = t.fid(*this);
            auto vs = oriented_tri_vertices(t);
            for (int j = 0; j < 3; j++) {
                F(i, j) = vs[j].vid(*this);
            }
        }

        return igl::write_triangle_mesh(path, V, F);
    }

    bool collapse_shortest(int target_vertex_count);

    bool collapse_qec();
    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh::Tuple& t);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh::Tuple& t);

    double compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple);

    void update_position(size_t v1, size_t v2, Tuple& new_vert);

    void move_vertex_attribute(size_t from, size_t to) override
    {
        m_vertex_positions[to] = m_vertex_positions[from];
    }

    void resize_attributes(size_t v, size_t e, size_t t) override { m_vertex_positions.resize(v); }
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
