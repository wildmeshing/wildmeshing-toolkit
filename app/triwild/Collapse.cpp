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

// every edge is collapsed, if it is shorter than 3/4 L

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

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

            if (length > 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l) return false;

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
    // record boundary vertex as fixed in vertex attribute for accurate collapse after boundary
    // operations
    if (is_boundary_vertex(t)) vertex_attrs[t.vid(*this)].fixed = 1;
    if (is_boundary_vertex(t.switch_vertex(*this)))
        vertex_attrs[t.switch_vertex(*this).vid(*this)].fixed = 1;
    if (m_bnd_freeze &&
        (vertex_attrs[t.vid(*this)].fixed || vertex_attrs[t.switch_vertex(*this).vid(*this)].fixed))
        return false;
    cache.local().v1 = t.vid(*this);
    cache.local().v2 = t.switch_vertex(*this).vid(*this);
    cache.local().partition_id = vertex_attrs[t.vid(*this)].partition_id;
    // get max_energy
    cache.local().max_energy = get_quality(t);
    auto tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : tris) {
        cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    }
    m_max_energy = cache.local().max_energy;
    return true;
}
bool TriWild::collapse_edge_after(const Tuple& t)
{
    // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    double length2 =
        (vertex_attrs[cache.local().v1].pos - vertex_attrs[cache.local().v2].pos).squaredNorm();

    auto v13d = m_triwild_displacement(
        vertex_attrs[cache.local().v1].pos(0),
        vertex_attrs[cache.local().v1].pos(1));
    auto v23d = m_triwild_displacement(
        vertex_attrs[cache.local().v2].pos(0),
        vertex_attrs[cache.local().v2].pos(1));
    length2 = (v23d - v13d).squaredNorm();

    Eigen::Vector2d p;
    if (vertex_attrs[cache.local().v1].fixed)
        p = vertex_attrs[cache.local().v1].pos;
    else if (vertex_attrs[cache.local().v2].fixed) {
        p = vertex_attrs[cache.local().v1].pos;
    } else
        p = (vertex_attrs[cache.local().v1].pos + vertex_attrs[cache.local().v2].pos) / 2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.local().partition_id;

    // enforce heuristic
    if (length2 < 4. / 5. * 4. / 5. * m_target_l * m_target_l) {
        return true;
    }
    // check quality
    auto tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : tris) {
        if (get_quality(tri) > cache.local().max_energy) return false;
    }
    return true;
}