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
    const std::map<size_t, Eigen::Vector3d>& current_positions,
    const std::map<size_t, Eigen::Vector3d>& target_positions,
    std::map<size_t, Eigen::Vector3d>& collision_free_positions)
{
    Eigen::MatrixXd vertices_incl_invalids;
    Eigen::MatrixXi faces_incl_invalids;
    mesh.export_seamless_mesh_3d(vertices_incl_invalids, faces_incl_invalids);

    Eigen::MatrixXd vertices_clean;
    Eigen::MatrixXi faces_clean;
    Eigen::MatrixXi map_old_to_new_v_ids;
    igl::remove_unreferenced(
        vertices_incl_invalids,
        faces_incl_invalids,
        vertices_clean,
        faces_clean,
        map_old_to_new_v_ids);

    Eigen::MatrixXi edges;
    igl::edges(faces_clean, edges);

    const ipc::CollisionMesh collisionMesh(vertices_clean, edges, faces_clean);

    for (const auto& [v_id_old, p_current] : current_positions) {
        const int v_id_new = map_old_to_new_v_ids(v_id_old, 0);
        if (v_id_new < 0) {
            continue;
        }
        vertices_clean.row(v_id_new) = p_current;
    }

    // create vertex matrix and adjust positions for target
    Eigen::MatrixXd vertices_targets = vertices_clean;
    for (const auto& [v_id_old, p_target] : target_positions) {
        const size_t v_id_new = map_old_to_new_v_ids(v_id_old, 0);
        vertices_targets.row(v_id_new) = p_target;
    }

    const double t =
        ipc::compute_collision_free_stepsize(collisionMesh, vertices_clean, vertices_targets);

    for (const auto& [v_id_old, p_target] : target_positions) {
        const int v_id_new = map_old_to_new_v_ids(v_id_old, 0);
        if (v_id_new < 0) {
            continue;
        }
        const Eigen::Vector3d p_current = vertices_clean.row(v_id_new);
        const Eigen::Vector3d p = (1 - t) * p_current + t * p_target;
        collision_free_positions[v_id_old] = p;
    }

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

    {
        Eigen::MatrixXd vertices_buf;
        Eigen::MatrixXi faces_buf;
        Eigen::MatrixXi I; // vector pointing from old to new vertex ids
        igl::remove_unreferenced(vertices, faces, vertices_buf, faces_buf, I);
        vertices = vertices_buf;
        faces = faces_buf;
    }

    Eigen::MatrixXi edges;
    igl::edges(faces, edges);

    const ipc::CollisionMesh collisionMesh(vertices, edges, faces);

    return ipc::has_intersections(collisionMesh, vertices);
}
} // namespace adaptive_tessellation