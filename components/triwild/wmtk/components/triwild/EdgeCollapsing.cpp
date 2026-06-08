#include "TriWildMesh.h"
#include "oneapi/tbb/concurrent_vector.h"
#include "wmtk/TriMesh.h"

#include <igl/Timer.h>
#include <algorithm>
#include <atomic>
#include <unordered_set>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::triwild {

void TriWildMesh::collapse_all_edges(bool is_limit_length)
{
    log_and_throw_error("edge collapsing is not implemented yet");
    // igl::Timer timer;
    // double time;
    // timer.start();

    // auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    // logger().info("#edges = {}", get_edges().size());
    // for (const Tuple& loc : get_edges()) {
    //     // collect all edges. Filtering too long edges happens in `is_weight_up_to_date`
    //     collect_all_ops.emplace_back("edge_collapse", loc);
    //     collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
    // }
    // auto collect_failure_ops = tbb::concurrent_vector<std::pair<std::string, Tuple>>();
    // std::atomic_int count_success = 0;
    // time = timer.getElapsedTime();
    // wmtk::logger().info("edge collapse prepare time: {:.4}s", time);
    // auto setup_and_execute = [&](auto& executor) {
    //     executor.renew_neighbor_tuples = [&](const auto& m, auto op, const auto& newts) {
    //         count_success++;
    //         std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
    //         for (auto t : newts) {
    //             op_tups.emplace_back(op, t);
    //             op_tups.emplace_back(op, t.switch_vertex(m));
    //         }
    //         return op_tups;
    //     };
    //     executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
    //     executor.num_threads = NUM_THREADS;
    //     executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
    //         auto& VA = m_vertex_attribute;
    //         auto& [weight, op, tup] = ele;
    //         auto length = m.get_length2(tup);
    //         if (length != -weight) return false;
    //         //
    //         size_t v1_id = tup.vid(*this);
    //         size_t v2_id = tup.switch_vertex(*this).vid(*this);
    //         double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
    //         if (is_limit_length && length > m_params.collapsing_l2 * sizing_ratio * sizing_ratio)
    //             return false;
    //         return true;
    //     };

    //     executor.on_fail = [&](auto& m, auto op, auto& t) {
    //         collect_failure_ops.emplace_back(op, t);
    //     };
    //     // Execute!!
    //     do {
    //         count_success.store(0, std::memory_order_release);
    //         wmtk::logger().info("Prepare to collapse {}", collect_all_ops.size());
    //         igl::Timer t1;
    //         t1.start();
    //         executor(*this, collect_all_ops);
    //         wmtk::logger().info("edge collapse execute time: {:.4}s", t1.getElapsedTimeInSec());
    //         wmtk::logger().info(
    //             "Collapsed {}, retrying failed {}",
    //             (int)count_success,
    //             collect_failure_ops.size());
    //         collect_all_ops.clear();
    //         for (auto& item : collect_failure_ops) collect_all_ops.emplace_back(item);
    //         collect_failure_ops.clear();
    //     } while (count_success.load(std::memory_order_acquire) > 0);
    // };
    // if (NUM_THREADS > 0) {
    //     timer.start();
    //     auto executor = wmtk::ExecutePass<TriWildMesh>(wmtk::ExecutionPolicy::kPartition);
    //     executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
    //         return m.try_set_edge_mutex_two_ring(e, task_id);
    //     };
    //     setup_and_execute(executor);
    //     time = timer.getElapsedTime();
    //     wmtk::logger().info("edge collapse operation time parallel: {:.4}s", time);
    // } else {
    //     timer.start();
    //     auto executor = wmtk::ExecutePass<TriWildMesh>(wmtk::ExecutionPolicy::kSeq);
    //     setup_and_execute(executor);
    //     time = timer.getElapsedTime();
    //     wmtk::logger().info("edge collapse operation time serial: {:.4}s", time);
    // }
}

bool TriWildMesh::collapse_edge_before(const Tuple& loc) // input is an edge
{
    return false;
}

bool TriWildMesh::collapse_edge_after(const Tuple& loc)
{
    return false;
}

} // namespace wmtk::components::triwild