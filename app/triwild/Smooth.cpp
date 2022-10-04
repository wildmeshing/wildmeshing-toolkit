
#include "TriWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
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
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);

    write_obj("smooth_after_1.obj");

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
        wmtk::AMIPS2D_energy,
        wmtk::AMIPS2D_jacobian,
        wmtk::AMIPS2D_hessian);

    // if vert is boundary project to the closest bounary edge
    std::vector<std::array<double, 4>> neighbor_assemble;
    // getting the boundary edges that point can be projected to, there are only 2 edges
    for (auto e : get_one_ring_edges_for_vertex(t)) {
        if (is_boundary_edge(e))
            neighbor_assemble.emplace_back(std::array<double, 4>{
                vertex_attrs[e.vid(*this)].pos(0),
                vertex_attrs[e.vid(*this)].pos(1),
                vertex_attrs[e.switch_vertex(*this).vid(*this)].pos(0),
                vertex_attrs[e.switch_vertex(*this).vid(*this)].pos(1)});
    }
    assert(neighbor_assemble.size() == 2);

    if (is_boundary_vertex(t)) {
        auto project = wmtk::try_project(vertex_attrs[t.vid(*this)].pos, neighbor_assemble);
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
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
