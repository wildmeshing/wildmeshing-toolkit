#include "EdgeOperations2d.h"
#include "spdlog/spdlog.h"

#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>

#include <wmtk/ExecutionScheduler.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace wmtk;
using namespace Edge2d;
auto unique_edge_tuples = [](const auto& m, auto& edges) {
    std::stable_sort(edges.begin(), edges.end(), [&](const auto& a, const auto& b) {
        return a.eid(m) < b.eid(m);
    }); // todo: use unique global id here would be very slow!

    edges.erase(
        std::unique(
            edges.begin(),
            edges.end(),
            [&](const auto& a, const auto& b) { return a.eid(m) == b.eid(m); }),
        edges.end());
};

std::vector<TriMesh::Tuple> Edge2d::EdgeOperations2d::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(), j));
        }
    }
    unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool Edge2d::EdgeOperations2d::collapse_shortest(int target_operation_count)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

    auto renew = [](auto& m, auto op, auto& tris) {
        auto edges = m.new_edges_after(tris);
        auto optup = std::vector<std::pair<std::string, Tuple>>();
        for (auto& e : edges) optup.emplace_back("edge_collapse", e);
        return optup;
    };
    auto measure_len2 = [](auto& m, auto op, const Tuple& new_e) {
        auto len2 =
            (m.m_vertex_positions[new_e.vid()] - m.m_vertex_positions[new_e.switch_vertex(m).vid()])
                .squaredNorm();
        return -len2;
    };
    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency = target_operation_count;
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    }
    return true;
}