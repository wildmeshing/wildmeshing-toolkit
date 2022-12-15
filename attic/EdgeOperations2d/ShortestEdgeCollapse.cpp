#include "EdgeOperations2d.h"

#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace wmtk;
using namespace Edge2d;

bool Edge2d::EdgeOperations2d::swap_edge_after(const TriMesh::Tuple& t)
{
    std::vector<TriMesh::Tuple> tris;
    tris.push_back(t);
    tris.push_back(t.switch_edge(*this));
    return true;
}

bool Edge2d::EdgeOperations2d::collapse_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;

    return true;
}

bool Edge2d::EdgeOperations2d::split_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.switch_vertex(*this).vid(*this);
    vertex_attrs[vid].pos = p;

    return true;
}

std::vector<TriMesh::Tuple> Edge2d::EdgeOperations2d::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool Edge2d::EdgeOperations2d::collapse_shortest(int target_operation_count)
{
    size_t initial_size = get_vertices().size();
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
            (m.vertex_attrs[new_e.vid(m)].pos - m.vertex_attrs[new_e.switch_vertex(m).vid(m)].pos)
                .squaredNorm();
        return -len2;
    };
    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        executor.stopping_criterion_checking_frequency = target_operation_count > 0
                                                             ? target_operation_count + 1
                                                             : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    }
    return true;
}