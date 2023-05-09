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
    for (int i = 0; i < vertices_incl_invalids.rows(); i++) {
        vertices_incl_invalids.row(i) = mesh.vertex_attrs[i].pos_world;
    }

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

    for (const auto& [v, p_target] : target_positions) {
        const Eigen::Vector3d p_current = current_positions.at(v);
        const Eigen::Vector3d p = (1 - t) * p_current + t * p_target;
        collision_free_positions[v] = p;
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
    for (int i = 0; i < vertices.rows(); i++) {
        vertices.row(i) = mesh.vertex_attrs[i].pos_world;
    }

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

inline void displace_self_intersection_free(AdaptiveTessellation& mesh)
{
    // set all world positions according to the position map
    Eigen::MatrixXd vertices_uv_with_seams;
    Eigen::MatrixXi faces;
    mesh.export_mesh(vertices_uv_with_seams, faces);

    Eigen::MatrixXd vertices_on_position_map;
    vertices_on_position_map.resize(vertices_uv_with_seams.rows(), 3);
    Eigen::MatrixXd vertices_displaced;
    vertices_displaced.resize(vertices_uv_with_seams.rows(), 3);
    for (int i = 0; i < vertices_uv_with_seams.rows(); i++) {
        const double& u = vertices_uv_with_seams(i, 0);
        const double& v = vertices_uv_with_seams(i, 1);
        vertices_on_position_map.row(i) = mesh.mesh_parameters.m_displacement->get_position(u, v);
        vertices_displaced.row(i) = mesh.mesh_parameters.m_displacement->get(u, v);
    }

    mesh.remove_seams(vertices_on_position_map, faces);
    mesh.remove_seams(vertices_displaced, faces);

    Eigen::MatrixXd vertices_non_intersecting;
    Eigen::MatrixXd vertices_target;
    Eigen::MatrixXi faces_clean;
    Eigen::MatrixXi map_old_to_new_v_ids;
    Eigen::MatrixXi map_new_to_old_v_ids;
    igl::remove_unreferenced(
        vertices_on_position_map,
        faces,
        vertices_non_intersecting,
        faces_clean,
        map_old_to_new_v_ids,
        map_new_to_old_v_ids);

    igl::remove_unreferenced(
        vertices_displaced,
        faces,
        vertices_target,
        faces_clean,
        map_old_to_new_v_ids);

    // check for self intersections - if there are any, find edges that self intersect and split
    // them until no more self intersections exist
    Eigen::MatrixXi edges;
    igl::edges(faces_clean, edges);
    const ipc::CollisionMesh collisionMesh(vertices_non_intersecting, edges, faces_clean);
    if (ipc::has_intersections(collisionMesh, vertices_non_intersecting)) {
        // TODO perform edge splits to remove self-intersections
        spdlog::warn("Mesh has self-self intersections even for 0 displacement. This case was not "
                     "handled yet.");
        return;
    }

    // move vertices towards displaced positions as far as possible
    const double t0 = ipc::compute_collision_free_stepsize(
        collisionMesh,
        vertices_non_intersecting,
        vertices_target);

    // move vertices
    for (int i = 0; i < vertices_non_intersecting.rows(); i++) {
        const Eigen::Vector3d& p0 = vertices_non_intersecting.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t0) * p0 + t0 * p1;
        vertices_non_intersecting.row(i) = p;
        mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
    }
    if (t0 == 1.0) {
        return;
    }

    // move only half the possible distance and iterate a few times
    for (int r = 0; r < 4; ++r) {
        for (int i = 0; i < vertices_non_intersecting.rows(); i++) {
            Eigen::MatrixXd buf = vertices_non_intersecting;
            buf.row(i) = vertices_target.row(i);
            const double t =
                0.5 *
                ipc::compute_collision_free_stepsize(collisionMesh, vertices_non_intersecting, buf);

            const Eigen::Vector3d& p0 = vertices_non_intersecting.row(i);
            const Eigen::Vector3d& p1 = vertices_target.row(i);
            const Eigen::Vector3d p = (1 - t) * p0 + t * p1;


            vertices_non_intersecting.row(i) = p;
            mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
        }
    }

    // move vertices as far as possible
    for (int i = 0; i < vertices_non_intersecting.rows(); i++) {
        Eigen::MatrixXd buf = vertices_non_intersecting;
        buf.row(i) = vertices_target.row(i);
        const double t =
            ipc::compute_collision_free_stepsize(collisionMesh, vertices_non_intersecting, buf);

        const Eigen::Vector3d& p0 = vertices_non_intersecting.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t) * p0 + t * p1;


        vertices_non_intersecting.row(i) = p;
        mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
    }
}
} // namespace adaptive_tessellation