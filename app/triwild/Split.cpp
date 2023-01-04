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

// every edge is split if it is longer than 4/5 L

auto split_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

void TriWild::split_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_split", t);
    wmtk::logger().info("=======split==========");

    wmtk::logger().info("size for edges to be split is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = split_renew;
        executor.priority = [&](auto& m, auto _, auto& e) { return m.get_length3d(e); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.get_length3d(tup);
            if (length != weight) return false;
            if (length < pow(4. / 3. * m.mesh_parameters.m_target_l, 2)) return false;
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
bool TriWild::split_edge_before(const Tuple& edge_tuple)
{
    if (!TriMesh::split_edge_before(edge_tuple)) return false;

    // check if the 2 vertices are on the same curve
    if (vertex_attrs[edge_tuple.vid(*this)].curve_id !=
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].curve_id)
        return false;

    cache.local().v1 = edge_tuple.vid(*this);
    cache.local().v2 = edge_tuple.switch_vertex(*this).vid(*this);
    cache.local().partition_id = vertex_attrs[edge_tuple.vid(*this)].partition_id;

    if (is_boundary_vertex(edge_tuple))
        vertex_attrs[cache.local().v1].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v1].boundary_vertex = false;
    if (is_boundary_vertex(edge_tuple.switch_vertex(*this)))
        vertex_attrs[cache.local().v2].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v2].boundary_vertex = false;

    // get max_energy // this isn't used for now. It is just purely heuristic
    cache.local().max_energy = get_quality(edge_tuple);
    auto tris = get_one_ring_tris_for_vertex(edge_tuple);
    for (auto tri : tris) {
        cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    }
    mesh_parameters.m_max_energy = cache.local().max_energy;
    return true;
}
bool TriWild::split_edge_after(const Tuple& edge_tuple)
{
    // adding heuristic decision. If length2 > 4. / 3. * 4. / 3. * m.m_target_l * m.m_target_l always split
    // transform edge length with displacement

    double length2 =
        (vertex_attrs[cache.local().v1].pos - vertex_attrs[cache.local().v2].pos).squaredNorm();

    auto v13d = mesh_parameters.m_triwild_displacement(
        vertex_attrs[cache.local().v1].pos(0),
        vertex_attrs[cache.local().v1].pos(1));
    auto v23d = mesh_parameters.m_triwild_displacement(
        vertex_attrs[cache.local().v2].pos(0),
        vertex_attrs[cache.local().v2].pos(1));
    length2 = (v23d - v13d).squaredNorm();

    const Eigen::Vector2d p =
        (vertex_attrs[cache.local().v1].pos + vertex_attrs[cache.local().v2].pos) / 2.0;
    auto vid = edge_tuple.switch_vertex(*this).vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.local().partition_id;
    vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;
    // take into account of periodicity
    if (vertex_attrs[cache.local().v1].boundary_vertex &&
        vertex_attrs[cache.local().v2].boundary_vertex) {
        vertex_attrs[vid].boundary_vertex = true;
        vertex_attrs[vid].t = std::fmod(
            (vertex_attrs[cache.local().v1].t + vertex_attrs[cache.local().v2].t) / 2.,
            mesh_parameters.m_boundary.m_arclengths[vertex_attrs[vid].curve_id].back());
    }
    for (auto e : get_one_ring_edges_for_vertex(edge_tuple.switch_vertex(*this))) {
        vertex_attrs[e.switch_vertex(*this).vid(*this)].boundary_vertex =
            is_boundary_vertex(e.switch_vertex(*this));
    }
    // enforce length check
    if (length2 > pow(4. / 3. * mesh_parameters.m_target_l, 2)) {
        return true;
    }

    // check quality // this reallyis not checked if the edge is longer than target
    // auto tris = get_one_ring_tris_for_vertex(t);
    // for (auto tri : tris) {
    //     if (get_quality(tri) > cache.local().max_energy) return false;
    // }

    return false;
}
