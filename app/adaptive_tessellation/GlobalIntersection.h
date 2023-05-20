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
inline double compute_collision_free_stepsize(const AdaptiveTessellation& mesh)
{
    spdlog::warn("This method was actually never tested! Handle with care!");
    Eigen::MatrixXd vertices_target;
    Eigen::MatrixXi faces_target;
    Eigen::MatrixXd vertices_texture;
    Eigen::MatrixXi faces_texture;
    mesh.export_mesh_with_displacement(
        vertices_target,
        faces_target,
        vertices_texture,
        faces_texture);

    Eigen::MatrixXd vertices_current;
    Eigen::MatrixXi faces;
    mesh.export_mesh_mapped_on_input(vertices_current, faces, vertices_texture, faces_texture);

    assert(faces == faces_target);

    // check for self intersections
    Eigen::MatrixXi edges;
    igl::edges(faces, edges);
    const ipc::CollisionMesh collisionMesh(vertices_current, edges, faces);
    if (ipc::has_intersections(collisionMesh, vertices_current)) {
        spdlog::warn("Mesh has self-intersections even for 0 displacement.");
        return 0;
    }

    // move vertices towards displaced positions as far as possible
    return ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, vertices_target);
}

/**
 * @brief Wrapper for has_intersection from the ipc toolkit.
 * @param mesh the AdaptiveTessellation mesh
 */
inline bool has_intersection(const AdaptiveTessellation& mesh)
{
    spdlog::warn("This method was actually never tested! Handle with care!");
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    Eigen::MatrixXd vertices_texture;
    Eigen::MatrixXi faces_texture;
    mesh.export_mesh_with_displacement(vertices, faces, vertices_texture, faces_texture);

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
    mesh.export_mesh_with_displacement(
        vertices_target,
        faces_target,
        vertices_texture,
        faces_texture);

    Eigen::MatrixXd vertices_current;
    Eigen::MatrixXi faces;
    mesh.export_mesh_mapped_on_input(vertices_current, faces, vertices_texture, faces_texture);

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
    const ipc::CollisionMesh collisionMesh(vertices_current, edges, faces);
    if (ipc::has_intersections(collisionMesh, vertices_current)) {
        // TODO perform edge splits to remove self-intersections
        spdlog::warn("Mesh has self-intersections even for 0 displacement. This case was not "
                     "handled yet.");
        return;
    }
    spdlog::info("No self intersections on start positions.");

    // move vertices towards displaced positions as far as possible
    const double t0 =
        ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, vertices_target);
    spdlog::info("t0 = {}", t0);

    // move vertices
    for (int i = 0; i < vertices_current.rows(); i++) {
        const Eigen::Vector3d& p0 = vertices_current.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t0) * p0 + t0 * p1;
        vertices_current.row(i) = p;
        update_pos_world(i, p);
    }
    if (t0 == 1.0) {
        return;
    }

    // move only half the possible distance and iterate a few times
    std::vector<bool> vertex_is_converged(vertices_current.rows(), false);
    // for (int r = 0; r < 4; ++r) {
    //     spdlog::info("Iteration {} of 4", r);
    //     for (int i = 0; i < vertices_current.rows(); i++) {
    //         if (vertex_is_converged[i]) {
    //             continue;
    //         }
    //         Eigen::MatrixXd buf = vertices_current;
    //         buf.row(i) = vertices_target.row(i);
    //         const double t =
    //             0.5 * ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, buf);
    //
    //         if (t == 0) {
    //             vertex_is_converged[i] = true;
    //             continue;
    //         }
    //
    //         const Eigen::Vector3d& p0 = vertices_current.row(i);
    //         const Eigen::Vector3d& p1 = vertices_target.row(i);
    //         const Eigen::Vector3d p = (1 - t) * p0 + t * p1;
    //
    //
    //         vertices_current.row(i) = p;
    //        // mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
    //        update_pos_world(i, p);
    //    }
    //}

    // move vertices as far as possible
    spdlog::info("Final vertex displacing.");
    for (int i = 0; i < vertices_current.rows(); i++) {
        if (vertex_is_converged[i]) {
            continue;
        }
        if (i % (vertices_current.rows() / 10) == 0) {
            double prcnt = 100 * static_cast<double>(i) / vertices_current.rows();
            spdlog::info("{}% done", prcnt);
        }
        Eigen::MatrixXd buf = vertices_current;
        buf.row(i) = vertices_target.row(i);
        const double t = ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, buf);

        const Eigen::Vector3d& p0 = vertices_current.row(i);
        const Eigen::Vector3d& p1 = vertices_target.row(i);
        const Eigen::Vector3d p = (1 - t) * p0 + t * p1;

        vertices_current.row(i) = p;
        update_pos_world(i, p);
    }
}
} // namespace adaptive_tessellation