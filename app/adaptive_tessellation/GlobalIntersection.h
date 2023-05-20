#pragma once
#include "AdaptiveTessellation.h"

#include <igl/edges.h>
#include <igl/remove_unreferenced.h>
#include <spdlog/stopwatch.h>
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

inline void move_subset_recursive(
    Eigen::MatrixXd& vertices_current,
    const Eigen::MatrixXd& vertices_target,
    const ipc::CollisionMesh& collisionMesh,
    const std::vector<size_t>& sorted_vertex_ids,
    const std::vector<bool>& vertex_is_active)
{
    Eigen::MatrixXd vertices_target_subset = vertices_current;
    for (size_t i = 0; i < sorted_vertex_ids.size(); ++i) {
        if (vertex_is_active[i]) {
            vertices_target_subset.row(sorted_vertex_ids[i]) =
                vertices_target.row(sorted_vertex_ids[i]);
        }
    }

    const double t = ipc::compute_collision_free_stepsize(
        collisionMesh,
        vertices_current,
        vertices_target_subset,
        ipc::DEFAULT_BROAD_PHASE_METHOD,
        0.0,
        ipc::DEFAULT_CCD_TOLERANCE,
        100);

    // reposition
    double t_interp = 0.99 * t;
    if (t == 1.0) {
        if (!ipc::has_intersections(collisionMesh, vertices_target_subset)) {
            t_interp = 1.0;
        }
    }
    for (size_t i = 0; i < vertices_current.rows(); ++i) {
        const Eigen::Vector3d& p0 = vertices_current.row(i);
        const Eigen::Vector3d& p1 = vertices_target_subset.row(i);
        const Eigen::Vector3d p = (1 - t_interp) * p0 + t_interp * p1;
        vertices_current.row(i) = p;
    }


    // compute subsets
    // get min and max true
    size_t subset_begin = -1;
    size_t subset_end = vertex_is_active.size();
    for (size_t i = 0; i < vertex_is_active.size(); ++i) {
        if (subset_begin == -1 && vertex_is_active[i]) {
            subset_begin = i;
        }
        if (subset_begin != -1 && !vertex_is_active[i]) {
            subset_end = i;
            break;
        }
    }

    if (t == 1.0) {
        // if (subset_end - subset_begin > 1) {
        //     spdlog::info(
        //         "Terminate for {} of {} vertices",
        //         (subset_end - subset_begin),
        //         vertex_is_active.size());
        // }
        return;
    }

    if (subset_end - subset_begin <= 1) {
        return;
    }
    //spdlog::info("Recursive CCD {}/{}.", (subset_end - subset_begin), vertex_is_active.size());

    const size_t subset_half = subset_begin + (subset_end - subset_begin) / 2;

    std::vector<bool> vertex_is_active_left(vertex_is_active.size(), false);
    for (size_t i = subset_begin; i < subset_half; ++i) {
        vertex_is_active_left[i] = true;
    }
    std::vector<bool> vertex_is_active_right(vertex_is_active.size(), false);
    for (size_t i = subset_half; i < subset_end; ++i) {
        vertex_is_active_right[i] = true;
    }

    move_subset_recursive(
        vertices_current,
        vertices_target,
        collisionMesh,
        sorted_vertex_ids,
        vertex_is_active_left);
    move_subset_recursive(
        vertices_current,
        vertices_target,
        collisionMesh,
        sorted_vertex_ids,
        vertex_is_active_right);
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

    if constexpr (false) {
        spdlog::info("Iterative per vertex CCD");
        // move vertices towards displaced positions as far as possible
        spdlog::stopwatch sw;
        const double t0 =
            ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, vertices_target);
        spdlog::info("Elapsed: {} seconds", sw);
        spdlog::info("t0 = {}", t0);

        // move vertices
        double t_interp = 0.99 * t0;
        if (t0 == 1.0) {
            if (!ipc::has_intersections(collisionMesh, vertices_target)) {
                t_interp = 1.0;
            }
        }
        for (int i = 0; i < vertices_current.rows(); i++) {
            const Eigen::Vector3d& p0 = vertices_current.row(i);
            const Eigen::Vector3d& p1 = vertices_target.row(i);
            const Eigen::Vector3d p = (1 - t_interp) * p0 + t_interp * p1;
            vertices_current.row(i) = p;
            update_pos_world(i, p);
        }
        if (t0 == 1.0) {
            return;
        }

        // move only half the possible distance and iterate a few times
        std::vector<bool> vertex_is_converged(vertices_current.rows(), false);
        if constexpr (false) {
            // move in half steps towards final position (very slow!)
            for (int r = 0; r < 4; ++r) {
                spdlog::info("Iteration {} of 4", r);
                for (int i = 0; i < vertices_current.rows(); i++) {
                    if (vertex_is_converged[i]) {
                        continue;
                    }
                    Eigen::MatrixXd buf = vertices_current;
                    buf.row(i) = vertices_target.row(i);
                    const double t =
                        0.5 *
                        ipc::compute_collision_free_stepsize(collisionMesh, vertices_current, buf);

                    if (t == 0) {
                        vertex_is_converged[i] = true;
                        continue;
                    }

                    const Eigen::Vector3d& p0 = vertices_current.row(i);
                    const Eigen::Vector3d& p1 = vertices_target.row(i);
                    const Eigen::Vector3d p = (1 - t) * p0 + t * p1;


                    vertices_current.row(i) = p;
                    // mesh.vertex_attrs[map_new_to_old_v_ids(i)].pos_world = p;
                    update_pos_world(i, p);
                }
            }
        }

        if constexpr (true) {
            // move vertices as far as possible
            spdlog::info("Final vertex displacing.");
            for (int i = 0; i < vertices_current.rows(); i++) {
                if (vertex_is_converged[i]) {
                    continue;
                }
                //if (i % (vertices_current.rows() / 10) == 0) {
                //    double prcnt = 100 * static_cast<double>(i) / vertices_current.rows();
                //    spdlog::info("{}% done", prcnt);
                //}
                Eigen::MatrixXd buf = vertices_current;
                buf.row(i) = vertices_target.row(i);
                // spdlog::stopwatch sw;
                const double t = ipc::compute_collision_free_stepsize(
                    collisionMesh,
                    vertices_current,
                    buf,
                    ipc::DEFAULT_BROAD_PHASE_METHOD,
                    0.0,
                    ipc::DEFAULT_CCD_TOLERANCE,
                    1000);
                // spdlog::info("Vertex {} - Elapsed: {} seconds", i, sw);

                const Eigen::Vector3d& p0 = vertices_current.row(i);
                const Eigen::Vector3d& p1 = vertices_target.row(i);
                const Eigen::Vector3d p = (1 - t) * p0 + t * p1;

                vertices_current.row(i) = p;
                update_pos_world(i, p);
            }
        }
    }

    if constexpr (true) {
        spdlog::info("Recursive CCD");
        // recursively try to move half of the mesh to target

        // sort by x
        std::multimap<double, size_t> map_x_to_v_id;
        for (size_t i = 0; i < vertices_current.rows(); ++i) {
            const double x = vertices_current(i, 0);
            map_x_to_v_id.insert({x, i});
        }
        std::vector<size_t> sorted_vertex_ids;
        sorted_vertex_ids.reserve(vertices_current.rows());
        for (const auto& [x, id] : map_x_to_v_id) {
            sorted_vertex_ids.emplace_back(id);
        }
        std::vector<bool> vertex_is_active(vertices_current.rows(), true);

        move_subset_recursive(
            vertices_current,
            vertices_target,
            collisionMesh,
            sorted_vertex_ids,
            vertex_is_active);

        for (size_t i = 0; i < vertices_current.rows(); ++i) {
            const Eigen::Vector3d& p = vertices_current.row(i);
            update_pos_world(i, p);
        }
    }
}
} // namespace adaptive_tessellation