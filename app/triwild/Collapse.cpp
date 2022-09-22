#include "TriWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <limits>
#include <optional>
using namespace triwild;
using namespace wmtk;

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

std::vector<TriMesh::Tuple> TriWild::new_edges_after(const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

void TriWild::collapse_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);
    wmtk::logger().info("=======collapse==========");

    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) { return -m.get_length2(e); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != -weight) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<TriWild, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TriWild, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}
bool TriWild::collapse_edge_before(const Tuple& t)
{
    if (!TriMesh::collapse_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].fixed || vertex_attrs[t.switch_vertex(*this).vid(*this)].fixed)
        return false;
    collapse_cache.local().v1 = t.vid(*this);
    collapse_cache.local().v2 = t.switch_vertex(*this).vid(*this);
    collapse_cache.local().partition_id = vertex_attrs[t.vid(*this)].partition_id;

    // get max_energy
    auto tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : tris) {
        collapse_cache.local().max_energy =
            std::max(collapse_cache.local().max_energy, get_quality(tri));
    }
    return true;
}
bool TriWild::collapse_edge_after(const Tuple& t)
{
    const Eigen::Vector2d p = (vertex_attrs[collapse_cache.local().v1].pos +
                               vertex_attrs[collapse_cache.local().v2].pos) /
                              2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = collapse_cache.local().partition_id;
    // check quality
    auto tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : tris) {
        if (get_quality(tri) > collapse_cache.local().max_energy) return false;
    }
    return true;
}