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


void TriWild::swap_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("=======swap==========");

    wmtk::logger().info("size for edges to be swap is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) { return m.get_length2(e); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) return false;
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
bool TriWild::swap_edge_before(const Tuple& t)
{
    if (!TriMesh::swap_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].fixed || vertex_attrs[t.switch_vertex(*this).vid(*this)].fixed)
        return false;

    // get max_energy
    cache.local().max_energy = -1;
    auto tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : tris) {
        assert(get_quality(tri) > 0);
        cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    }
    max_energy = cache.local().max_energy;
    return true;
}
bool TriWild::swap_edge_after(const Tuple& t)
{
    // check quality
    auto tris = get_one_ring_tris_for_vertex(t);

    for (auto tri : tris) {
        if (get_quality(tri) >= cache.local().max_energy) return false;
        if (get_quality(tri) < 0) return false; // reject operations that cause triangle inversion
    }
    return true;
}