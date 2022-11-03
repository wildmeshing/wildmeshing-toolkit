
#include "TriWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>


#include <limits>
#include <optional>
std::function<double(const std::array<double, 6>&)> AMIPS_auto_value = [](auto& T) {
    return wmtk::AMIPS_autodiff(T).getValue();
};
std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> AMIPS_auto_grad =
    [](auto& T, auto& G) { G = wmtk::AMIPS_autodiff(T).getGradient(); };
std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&)> AMIPS_auto_hessian =
    [](auto& T, auto& H) { H = wmtk::AMIPS_autodiff(T).getHessian(); };

std::function<double(const std::array<double, 6>&)> SymDI_auto_value = [](auto& T) {
    return wmtk::SymDi_autodiff(T).getValue();
};
std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> SymDI_auto_grad =
    [](auto& T, auto& G) { G = wmtk::SymDi_autodiff(T).getGradient(); };
std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&)> SymDi_auto_hessian =
    [](auto& T, auto& H) { H = wmtk::SymDi_autodiff(T).getHessian(); };


bool triwild::TriWild::smooth_before(const Tuple& t)
{
    if (m_bnd_freeze && vertex_attrs[t.vid(*this)].fixed) return false;
    return true;
}


bool triwild::TriWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);

    // write_obj("smooth_after_1.obj");

    // Computes the maximal error around the one ring
    // that is needed to ensure the operation will decrease the error measure
    auto max_quality = 0.;
    for (auto& tri : locs) {
        max_quality = std::max(max_quality, get_quality(tri));
    }
    assert(max_quality > 0); // If max quality is zero it is likely that the triangles are flipped

    m_max_energy = max_quality;

    // Collects the coordinate of all vertices in the 1-ring
    std::vector<std::array<double, 6>> assembles(locs.size());
    auto loc_id = 0;

    // For each triangle, make a reordered copy of the vertices so that
    // the vertex to optimize is always the first
    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.fid(*this);

        assert(!is_inverted(loc));
        auto local_tuples = oriented_tri_vertices(loc);
        std::array<size_t, 3> local_verts;
        for (auto i = 0; i < 3; i++) {
            local_verts[i] = local_tuples[i].vid(*this);
        }

        local_verts = wmtk::orient_preserve_tri_reorder(local_verts, vid);

        for (auto i = 0; i < 3; i++) {
            for (auto j = 0; j < 2; j++) {
                T[i * 2 + j] = vertex_attrs[local_verts[i]].pos[j];
            }
        }
        loc_id++;
    }

    // Make a backup of the current configuration
    auto old_pos = vertex_attrs[vid].pos;
    auto old_asssembles = assembles;

    // Minimize distortion using newton's method
    vertex_attrs[vid].pos = wmtk::newton_method_from_stack_2d(
        assembles,
        AMIPS_auto_value,
        AMIPS_auto_grad,
        AMIPS_auto_hessian);

    // if it is a boundary vertex, project the vertex to the closest point on the boundary
    if (is_boundary_vertex(t)) {
        vertex_attrs[vid].pos = this->m_get_closest_point(vertex_attrs[vid].pos);
    }
    // get all the one-ring tris and if they are out of envelop reject the operation
    auto new_tris = get_one_ring_tris_for_vertex(t);
    if (!invariants(new_tris)) return false;
    // // Logging
    // wmtk::logger().info(
    //     "old pos {} -> new pos {}",
    //     old_pos.transpose(),
    //     vertex_attrs[vid].pos.transpose());
    return true;
}

void triwild::TriWild::smooth_all_vertices()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("vertex smoothing prepare time: {}s", time);
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TriWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TriWild, wmtk::ExecutionPolicy::kSeq>();
        bool moved = 0;
        int itr = 0;
        do {
            std::vector<Eigen::Vector2d> old_pos(vert_capacity());
            for (auto& v : get_vertices()) {
                old_pos[v.vid(*this)] = vertex_attrs[v.vid(*this)].pos;
            }
            executor(*this, collect_all_ops);
            write_obj("smooth" + std::to_string(itr) + ".obj");
            std::vector<Tuple> verts = get_vertices();
            for (int i = 0; i < vert_capacity() && !moved; i++) {
                auto vid = verts[i].vid(*this);
                moved |= ((old_pos[vid] - vertex_attrs[vid].pos).norm() < 1e-5);
            }
            itr++;
        } while (moved);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}