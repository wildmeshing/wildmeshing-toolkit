#pragma once
#include "AdaptiveTessellation.h"

#include <igl/edges.h>
#include <igl/remove_unreferenced.h>
#include <ipc/ipc.hpp>

namespace adaptive_tessellation {
/**
 * @brief Wrapper for compute_collision_free_stepsize from the ipc toolkit.
 * @param mesh The mesh for which an operation should be performed
 * @param vn The new vertex positions after the operation
 */
inline double compute_collision_free_stepsize(
    const AdaptiveTessellation& mesh,
    const Eigen::MatrixXd& vn)
{
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    mesh.export_seamless_mesh_3d(vertices, faces);
    Eigen::MatrixXi edges;
    igl::edges(faces, edges);

    const ipc::CollisionMesh collisionMesh(vertices, edges, faces);

    assert(vn.rows() == vertices.rows());
    assert(vn.cols() == vertices.cols());

    const double t = ipc::compute_collision_free_stepsize(collisionMesh, vertices, vn);
    return t;
}

/**
 * @brief Wrapper for has_intersection from the ipc toolkit.
 * @param mesh the AdaptiveTessellation mesh
 */
inline bool has_intersection(const AdaptiveTessellation& mesh)
{
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    mesh.export_seamless_mesh_3d(vertices, faces);

    Eigen::MatrixXd vertices2;
    Eigen::MatrixXi faces2;
    Eigen::MatrixXi I;
    igl::remove_unreferenced(vertices, faces, vertices2, faces2, I);

    Eigen::MatrixXi edges;
    igl::edges(faces2, edges);

    const ipc::CollisionMesh collisionMesh(vertices2, edges, faces2);

    return ipc::has_intersections(collisionMesh, vertices2);
}
} // namespace adaptive_tessellation