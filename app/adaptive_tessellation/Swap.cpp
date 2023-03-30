#include "AdaptiveTessellation.h"
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
using namespace adaptive_tessellation;
using namespace wmtk;


namespace {
class AdaptiveTessellationSwapEdgeOperation : public wmtk::TriMeshOperationShim<
                                                  AdaptiveTessellation,
                                                  AdaptiveTessellationSwapEdgeOperation,
                                                  wmtk::TriMeshSwapEdgeOperation>
{
public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t)
    {
        return wmtk::TriMeshSwapEdgeOperation::execute(m, t);
    }
    bool before(AdaptiveTessellation& m, const Tuple& t)
    {
        if (wmtk::TriMeshSwapEdgeOperation::before(m, t)) {
            return true;
            //return  m.swap_before(t);
        }
        return false;
    }
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::after(m, ret_data)) {
            return true;
            //ret_data.success |= m.swap_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSwapEdgeOperation::invariants(m, ret_data)) {
            ret_data.success |= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};

    template <typename Executor>
    void addCustomOps(Executor& e) {

        e.add_operation(std::make_shared<AdaptiveTessellationSwapEdgeOperation>());
    }


auto swap_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};
auto swap_cost = [](auto& m, const TriMesh::Tuple& t) {
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    valences[0] = std::make_pair(t, m.get_valence_for_vertex(t));
    auto t2 = t.switch_vertex(m);
    valences[1] = std::make_pair(t2, m.get_valence_for_vertex(t2));
    auto t3 = (t.switch_edge(m)).switch_vertex(m);
    valences[2] = std::make_pair(t3, m.get_valence_for_vertex(t3));

    if ((t.switch_face(m)).has_value()) {
        auto t4 = (((t.switch_face(m)).value()).switch_edge(m)).switch_vertex(m);
        valences.emplace_back(t4, m.get_valence_for_vertex(t4));
    }
    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        int val = 6;
        auto one_ring_edges = m.get_one_ring_edges_for_vertex(vert);
        for (auto edge : one_ring_edges) {
            if (m.is_boundary_edge(edge)) {
                val = 4;
                break;
            }
        }
        cost_before_swap += (double)(valences[i].second - val) * (valences[i].second - val);
        cost_after_swap +=
            (i < 2) ? (double)(valences[i].second - 1 - val) * (valences[i].second - 1 - val)
                    : (double)(valences[i].second + 1 - val) * (valences[i].second + 1 - val);
    }
    return (cost_before_swap - cost_after_swap);
};

auto swap_accuracy_cost = [](auto& m, const TriMesh::Tuple& e) {
    auto e_before = m.get_accuracy_error(e.vid(m), e.switch_vertex(m).vid(m));
    if ((e.switch_face(m)).has_value()) {
        auto t4 = (((e.switch_face(m)).value()).switch_edge(m)).switch_vertex(m);
        auto t3 = (e.switch_edge(m)).switch_vertex(m);
        auto e_after = m.get_accuracy_error(t3.vid(m), t4.vid(m));
        return (e_before - e_after);
    } else
        return 0.;
};
}

void AdaptiveTessellation::swap_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("=======swap==========");

    wmtk::logger().info("size for edges to be swap is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addCustomOps(executor);
        executor.renew_neighbor_tuples = swap_renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            if (m.mesh_parameters.m_accuracy)
                return swap_accuracy_cost(m, e);
            else
                return swap_cost(m, e);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            if (m.mesh_parameters.m_accuracy) {
                auto cost = swap_accuracy_cost(m, tup);
                if (cost < m.mesh_parameters.m_accuracy_threshold ||
                    std::pow(cost - weight, 2) > 1e-5)
                    return false;
            } else {
                auto cost = swap_cost(m, tup);
                if (cost < 1e-5 || std::pow(cost - weight, 2) > 1e-5) return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}
bool AdaptiveTessellation::swap_edge_before(const Tuple& t)
{

    if (is_boundary_edge(t)) return false;
    if (is_boundary_vertex(t))
        vertex_attrs[cache.local().v1].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v1].boundary_vertex = false;
    if (is_boundary_vertex(t.switch_vertex(*this)))
        vertex_attrs[cache.local().v2].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v2].boundary_vertex = false;
    // // get max_energy
    // cache.local().max_energy = -1;
    // auto tris = get_one_ring_tris_for_vertex(t);
    // for (auto tri : tris) {
    //     assert(get_quality(tri) > 0);
    //     cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    // }
    // m_max_energy = cache.local().max_energy;
    return true;
}
bool AdaptiveTessellation::swap_edge_after([[maybe_unused]] const Tuple& t)
{
    // check quality and degenerate
    // auto tris = get_one_ring_tris_for_vertex(t);

    // for (auto tri : tris) {
    //     // if (get_quality(tri) >= cache.local().max_energy) return false; // this is commented out for energy check. Only check valence for now
    //     if (get_quality(tri) < 0) return false; // reject operations that cause triangle inversion
    // }
    return true;
}
