#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"


// get the quadrix in form of an array of 10 floating point numbers
Eigen::MatrixXd Edge2d::EdgeOperations2d::compute_Q_f(wmtk::TriMesh::Tuple& f_tuple)
{
    auto conn_indices = oriented_tri_vertices(f_tuple);
    Eigen::Vector3d A = vertex_attrs[conn_indices[0].vid()].pos;
    Eigen::Vector3d B = vertex_attrs[conn_indices[1].vid()].pos;
    Eigen::Vector3d C = vertex_attrs[conn_indices[2].vid()].pos;

    Eigen::Vector3d n = ((A - B).cross(C - B)).normalized();
    Eigen::Vector4d p;
    p(0) = n(0);
    p(1) = n(1);
    p(2) = n(2);
    p(3) = -n.dot(B);

    return p * p.transpose();
}

Eigen::MatrixXd Edge2d::EdgeOperations2d::compute_Q_v(wmtk::TriMesh::Tuple& v_tuple)
{
    auto conn_tris = get_one_ring_tris_for_vertex(v_tuple);
    Eigen::MatrixXd Q{};
    for (TriMesh::Tuple tri : conn_tris) {
        Eigen::MatrixXd Q_t = compute_Q_f(tri);
        Q += Q_t;
    }
    return Q;
}

double Edge2d::EdgeOperations2d::compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple)
{
    Eigen::Vector3d v = vertex_attrs[v_tuple.vid()].pos;
    Eigen::MatrixXd Q = compute_Q_v(v_tuple);
    Eigen::Vector4d t(0.0, 0.0, 0.0, 1.0);


    assert(false);
    return 0;
}

bool Edge2d::EdgeOperations2d::collapse_qec(int target)
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
