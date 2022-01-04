#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeCollapse.h"
void add_Qs(std::array<double, 10>& Q1, const std::array<double, 10>& Q2)
{
    for (int j = 0; j < 10; j++) {
        Q1[j] = Q1[j] + Q2[j];
    }
}

// get the quadrix in form of an array of 10 floating point numbers
std::array<double, 10> Edge2d::EdgeCollapse::compute_Q_f(wmtk::TriMesh::Tuple& f_tuple)
{
    auto conn_indices = get_oriented_vertices_for_tri(f_tuple);
    Eigen::Vector3d A = m_vertex_positions[conn_indices[0].get_vid()];
    Eigen::Vector3d B = m_vertex_positions[conn_indices[1].get_vid()];
    Eigen::Vector3d C = m_vertex_positions[conn_indices[2].get_vid()];

    Eigen::Vector3d n = ((A - B).cross(C - B)).normalized();
    double a = n(0);
    double b = n(1);
    double c = n(2);
    double d = -n.dot(B);

    std::array<double, 10> Q;
    // a^2, ab, ac, ad, b^2, bc, bd, c^2, cd, d^2,
    Q[0] = a * a;
    Q[1] = a * b;
    Q[2] = a * c;
    Q[3] = a * d;
    Q[4] = b * b;
    Q[5] = b * c;
    Q[6] = b * d;
    Q[7] = c * c;
    Q[8] = c * d;
    Q[9] = d * d;
    return Q;
}

std::array<double, 10> Edge2d ::EdgeCollapse::compute_Q_v(wmtk::TriMesh::Tuple& v_tuple)
{
    auto conn_tris = get_one_ring_tris_for_vertex(v_tuple);
    std::array<double, 10> Q{};
    for (TriMesh::Tuple tri : conn_tris) {
        std::array<double, 10> Q_t = compute_Q_f(tri);
        add_Qs(Q, Q_t);
    }
    return Q;
}

double Edge2d::EdgeCollapse::compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple)
{
    Eigen::Vector3d v = m_vertex_positions[v_tuple.get_vid()];
    std::array<double, 10> Q = compute_Q_v(v_tuple);

    double cost;
    return cost;
}

bool Edge2d::EdgeCollapse::collapse_qec()
{
    // find the valid pairs (for each vertex)
    std::vector<TriMesh::Tuple> edges = get_edges();
    double shortest = std::numeric_limits<double>::max();

    // find the best pair by keeping the priority queue
    // always keep the high cost one on top of the queue


    // do the edge collapse of the top pair

    // update cost priority queue
    return true;
}
