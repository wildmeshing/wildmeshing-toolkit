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
    mesh.export_seamless_mesh_with_displacement(vertices_incl_invalids, faces_incl_invalids);
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
    mesh.export_seamless_mesh_with_displacement(vertices, faces);
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
    spdlog::info("Compute self-intersection free displacement");
    Eigen::MatrixXd vertices_target;
    Eigen::MatrixXi faces_target;
    Eigen::MatrixXd vertices_texture;
    Eigen::MatrixXi faces_texture;
    mesh.export_mesh(vertices_target, faces_target, vertices_texture, faces_texture);

    Eigen::MatrixXd vertices_start;
    Eigen::MatrixXi faces;
    mesh.export_mesh_mapped_on_input(vertices_start, faces, vertices_texture, faces_texture);

    assert(faces == faces_target);

    std::map<size_t, std::set<size_t>> world_to_uv_ids;
    for (Eigen::Index i = 0; i < faces.rows(); ++i) {
        for (Eigen::Index j = 0; j < faces.cols(); ++j) {
            world_to_uv_ids[faces(i, j)].insert(faces_texture(i, j));
        }
    }

    auto update_pos_world = [&world_to_uv_ids, &mesh](const size_t i, const Eigen::Vector3d& p) {
        for (const size_t idx : world_to_uv_ids[i]) {
            mesh.vertex_attrs[idx].pos_world = p;
        }
    };

    // check for self intersections - if there are any, find edges that self intersect and split
    // them until no more self intersections exist
    Eigen::MatrixXi edges;
    igl::edges(faces, edges);
    const ipc::CollisionMesh collisionMesh(vertices_start, edges, faces);
    if (ipc::has_intersections(collisionMesh, vertices_start)) {
        // TODO perform edge splits to remove self-intersections
        spdlog::warn("Mesh has self-self intersections even for 0 displacement. This case was not "
                     "handled yet.");
        return;
    }
    spdlog::info("No self intersections on start positions.");

    // move vertices towards displaced positions as far as possible
    const double t0 =
        ipc::compute_collision_free_stepsize(collisionMesh, vertices_start, vertices_target);
    spdlog::info("t0 = {}", t0);

    // move vertices
    for (int i = 0; i < vertices_start.rows(); i++) {
        const Eigen::Vector3d& p0 = vertices_start.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t0) * p0 + t0 * p1;
        vertices_start.row(i) = p;
        // mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
        update_pos_world(i, p);
    }
    if (t0 == 1.0) {
        return;
    }

    // move only half the possible distance and iterate a few times
    std::vector<bool> vertex_is_converged(vertices_start.rows(), false);
    for (int r = 0; r < 4; ++r) {
        spdlog::info("Iteration {} of 4", r);
        for (int i = 0; i < vertices_start.rows(); i++) {
            if (vertex_is_converged[i]) {
                continue;
            }
            Eigen::MatrixXd buf = vertices_start;
            buf.row(i) = vertices_target.row(i);
            const double t =
                0.5 * ipc::compute_collision_free_stepsize(collisionMesh, vertices_start, buf);

            if (t == 0) {
                vertex_is_converged[i] = true;
                continue;
            }

            const Eigen::Vector3d& p0 = vertices_start.row(i);
            const Eigen::Vector3d& p1 = vertices_target.row(i);
            const Eigen::Vector3d p = (1 - t) * p0 + t * p1;


            vertices_start.row(i) = p;
            // mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
            update_pos_world(i, p);
        }
    }

    // move vertices as far as possible
    spdlog::info("Final vertex displacing.");
    for (int i = 0; i < vertices_start.rows(); i++) {
        if (vertex_is_converged[i]) {
            continue;
        }
        Eigen::MatrixXd buf = vertices_start;
        buf.row(i) = vertices_target.row(i);
        const double t = ipc::compute_collision_free_stepsize(collisionMesh, vertices_start, buf);

        const Eigen::Vector3d& p0 = vertices_start.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t) * p0 + t * p1;


        vertices_start.row(i) = p;
        // mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
        update_pos_world(i, p);
    }
}
} // namespace adaptive_tessellation