
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

bool triwild::TriWild::smooth_before(const Tuple& t)
{
    if (m_bnd_freeze && vertex_attrs[t.vid(*this)].fixed) return false;
    return true;
}

bool triwild::TriWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing with index.");
    auto vid = t.vid(*this);
    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);

    // Computes the maximal error around the one ring
    // that is needed to ensure the operation will decrease the error measure
    auto avg_quality = 0.;
    for (auto& tri : locs) {
        avg_quality += get_quality(tri);
    }
    avg_quality /= locs.size();
    assert(avg_quality > 0); // If max quality is zero it is likely that the triangles are flipped

    // m_max_energy = avg_quality;

    // getting the assembles
    //( one ring triangle vertex position in stack with last entry as the local vid of the smoothing
    // vertex)
    double idx = -1.;
    std::vector<std::array<double, 7>> assembles;
    for (auto tri : locs) {
        assert(!is_inverted(tri));
        std::array<double, 7> T;
        auto local_tuples = oriented_tri_vertices(tri);

        for (auto i = 0; i < 3; i++) {
            T[i * 2] = vertex_attrs[local_tuples[i].vid(*this)].pos(0);
            T[i * 2 + 1] = vertex_attrs[local_tuples[i].vid(*this)].pos(1);
            if (local_tuples[i].vid(*this) == vid) idx = (double)i;
        }
        assert(idx != -1);
        T[6] = idx;
        assembles.emplace_back(T);
    }

    // use a general root finding method that defaults to newton but if not changeing the position,
    // try gradient descent
    auto old_pos = vertex_attrs[vid].pos;
    vertex_attrs[vid].pos =
        wmtk::newton_method_with_fallback(this->m_target_l, assembles, *m_energy);

    // check boundary and project
    if (is_boundary_vertex(t)) {
        vertex_attrs[vid].pos = this->m_get_closest_point(vertex_attrs[vid].pos);
    }
    // get one-ring trinagles for new_tris
    auto new_tris = get_one_ring_tris_for_vertex(t);

    // check invariants
    if (!invariants(new_tris)) {
        vertex_attrs[vid].pos = old_pos;
        return false;
    }
    // check if energy is lowered
    double new_avg_quality = 0.;
    for (auto tri : new_tris) new_avg_quality += get_quality(tri);
    new_avg_quality /= locs.size();

    if (new_avg_quality > avg_quality) return false;

    assert(invariants(new_tris));
    return true;
}

void triwild::TriWild::smooth_all_vertices()
{
    wmtk::logger().info("=======smooth==========");
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
        bool nochange = 1;
        int itr = 0;
        do {
            nochange = 1;
            std::vector<Eigen::Vector2d> old_pos(vert_capacity());
            for (auto& v : get_vertices()) {
                old_pos[v.vid(*this)] = vertex_attrs[v.vid(*this)].pos;
            }
            executor(*this, collect_all_ops);
            // write_obj("smooth" + std::to_string(itr) + ".obj");
            std::vector<Tuple> verts = get_vertices();
            for (int i = 0; i < vert_capacity() && nochange; i++) {
                auto vid = verts[i].vid(*this);
                nochange &= ((old_pos[vid] - vertex_attrs[vid].pos).norm() < 1e-2);
            }
            itr++;
        } while (!nochange && itr < 10);
        wmtk::logger().info(itr);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}