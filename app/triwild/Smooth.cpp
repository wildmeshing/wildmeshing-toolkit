
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

namespace {

using namespace triwild;
using namespace wmtk;

class TriWildSmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                                  TriWild,
                                                  TriWildSmoothVertexOperation,
                                                  wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(const Tuple& t, TriWild& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::execute(t, m);
    }
    bool before_check(const Tuple& t, TriWild& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::before_check(t, m) && m.smooth_before(t);
    }
    bool after_check(const ExecuteReturnData& ret_data, TriWild& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::after_check(ret_data, m) &&
               m.smooth_after(ret_data.tuple);
    }
    bool invariants(const ExecuteReturnData& ret_data, TriWild& m)
    {
        return wmtk::TriMeshSmoothVertexOperation::invariants(ret_data, m) &&
               m.invariants(ret_data.new_tris);
    }
};

    template <typename Executor>
    void addCustomOps(Executor& e) {

        e.add_operation(std::make_shared<TriWildSmoothVertexOperation>());
    }
}

bool triwild::TriWild::smooth_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].fixed)
        return false;
    return true;
}


bool triwild::TriWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);


    // Computes the maximal error around the one ring
    // that is needed to ensure the operation will decrease the error measure
    auto max_quality = 0.;
    for (auto& tri : locs) {
        max_quality = std::max(max_quality, get_quality(tri));
    }

    assert(max_quality > 0); // If max quality is zero it is likely that the triangles are flipped

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

    // Logging
    wmtk::logger().info(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        vertex_attrs[vid].pos.transpose());

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
        addCustomOps(executor);
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TriWild, wmtk::ExecutionPolicy::kSeq>();
        addCustomOps(executor);
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
