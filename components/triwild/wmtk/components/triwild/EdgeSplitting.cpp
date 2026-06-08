#include "TriWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::triwild {

void TriWildMesh::split_all_edges()
{
    log_and_throw_error("edge splitting is not implemented yet");
    // igl::Timer timer;
    // double time;
    // timer.start();
    // auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    // for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);
    // time = timer.getElapsedTime();
    // wmtk::logger().info("edge split prepare time: {:.4}s", time);
    // auto setup_and_execute = [&](auto& executor) {
    //     executor.renew_neighbor_tuples = wmtk::renewal_simple;

    //     executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
    //     executor.num_threads = NUM_THREADS;
    //     executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
    //         auto [weight, op, tup] = ele;
    //         auto length = m.get_length2(tup);
    //         if (length != weight) return false;
    //         //
    //         size_t v1_id = tup.vid(*this);
    //         size_t v2_id = tup.switch_vertex(*this).vid(*this);
    //         double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
    //                                m_vertex_attribute[v2_id].m_sizing_scalar) /
    //                               2;
    //         if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) return false;
    //         return true;
    //     };
    //     executor(*this, collect_all_ops);
    // };
    // if (NUM_THREADS > 0) {
    //     timer.start();
    //     auto executor = wmtk::ExecutePass<TriWildMesh>(wmtk::ExecutionPolicy::kPartition);
    //     executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
    //         return m.try_set_edge_mutex_two_ring(e, task_id);
    //     };
    //     setup_and_execute(executor);
    //     time = timer.getElapsedTime();
    //     wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    // } else {
    //     timer.start();
    //     auto executor = wmtk::ExecutePass<TriWildMesh>(wmtk::ExecutionPolicy::kSeq);
    //     setup_and_execute(executor);
    //     time = timer.getElapsedTime();
    //     wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    // }
}

bool TriWildMesh::split_edge_before(const Tuple& loc0)
{
    log_and_throw_error("split_edge_before not implemented yet");
}

bool TriWildMesh::split_edge_after(const Tuple& loc)
{
    log_and_throw_error("split_edge_after not implemented yet");
}

} // namespace wmtk::components::triwild