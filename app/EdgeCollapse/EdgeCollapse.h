#include <wmtk/TriMesh.h>
#include <wmtk/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#pragma once

namespace Edge2d {

class EdgeCollapse : public wmtk::TriMesh
{
public:
    std::vector<Eigen::Vector3d> m_vertex_positions;

    EdgeCollapse(std::vector<Eigen::Vector3d> _m_vertex_positions)
        : m_vertex_positions(_m_vertex_positions)
    {}

    ~EdgeCollapse() {}

    bool collapse_shortest();

    bool collapse_qec();
    // get the quadrix in form of an array of 10 floating point numbers
    std::array<double, 10> compute_Q_f(wmtk::TriMesh::Tuple& t);

    std::array<double, 10> compute_Q_v(wmtk::TriMesh::Tuple& t);

    void update_position(size_t v1, size_t v2, Tuple& new_vert);
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
