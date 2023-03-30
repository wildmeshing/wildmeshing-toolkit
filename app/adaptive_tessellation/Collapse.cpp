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
using namespace wmtk;
using namespace adaptive_tessellation;

// every edge is collapsed, if it is shorter than 3/4 L

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

namespace {
class AdaptiveTessellationEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                                  AdaptiveTessellation,
                                                  AdaptiveTessellationEdgeCollapseOperation,
                                                  wmtk::TriMeshEdgeCollapseOperation>
{
public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(m, t);
    }
    bool before(AdaptiveTessellation& m, const Tuple& t)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
            return  m.collapse_edge_before(t);
        }
        return false;
    }
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::after(m, ret_data)) {
            ret_data.success |= m.collapse_edge_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshEdgeCollapseOperation::invariants(m, ret_data)) {
            ret_data.success |= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};

    template <typename Executor>
    void addCustomOps(Executor& e) {

        e.add_operation(std::make_shared<AdaptiveTessellationEdgeCollapseOperation>());
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

void AdaptiveTessellation::collapse_all_edges()
{
    for (auto f : get_faces()) assert(!is_inverted(f));
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);
    wmtk::logger().info("=======collapse==========");
    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            return -m.mesh_parameters.m_get_length(e.vid(m), e.switch_vertex(m).vid(m));
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.mesh_parameters.m_get_length(tup.vid(m), tup.switch_vertex(m).vid(m));
            if (length != -weight) return false;

            if (length > (4. / 5. * m.mesh_parameters.m_target_l)) return false;

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
bool AdaptiveTessellation::collapse_edge_before(const Tuple& edge_tuple)
{

    // check if the two vertices to be split is of the same curve_id
    if (vertex_attrs[edge_tuple.vid(*this)].curve_id !=
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].curve_id)
        return false;

    double length3d = mesh_parameters.m_get_length(
        edge_tuple.vid(*this),
        edge_tuple.switch_vertex(*this).vid(*this));
    // enforce heuristic
    assert(length3d < 4. / 5. * mesh_parameters.m_target_l);

    // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse after
    // boundary operations

    if (is_boundary_vertex(edge_tuple))
        vertex_attrs[edge_tuple.vid(*this)].boundary_vertex = true;
    else
        vertex_attrs[edge_tuple.vid(*this)].boundary_vertex = false;
    if (is_boundary_vertex(edge_tuple.switch_vertex(*this)))
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex = true;
    else
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex = false;
    if (mesh_parameters.m_bnd_freeze &&
        (vertex_attrs[edge_tuple.vid(*this)].boundary_vertex ||
         vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex))
        return false;
    cache.local().v1 = edge_tuple.vid(*this);
    cache.local().v2 = edge_tuple.switch_vertex(*this).vid(*this);
    cache.local().partition_id = vertex_attrs[edge_tuple.vid(*this)].partition_id;
    // get max_energy
    cache.local().max_energy = get_quality(edge_tuple);
    auto tris = get_one_ring_tris_for_vertex(edge_tuple);
    for (auto tri : tris) {
        cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    }
    mesh_parameters.m_max_energy = cache.local().max_energy;

    return true;
}
bool AdaptiveTessellation::collapse_edge_after(const Tuple& edge_tuple)
{
    // check if the both of the 2 vertices are fixed
    // if yes, then collapse is rejected
    if (vertex_attrs[cache.local().v1].fixed && vertex_attrs[cache.local().v2].fixed) return false;

    // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    double length3d = mesh_parameters.m_get_length(cache.local().v1, cache.local().v2);

    auto vid = edge_tuple.vid(*this);
    Eigen::Vector2d p;
    double t_parameter;
    double mod_length =
        mesh_parameters.m_boundary.m_arclengths[vertex_attrs[edge_tuple.vid(*this)].curve_id]
            .back();

    if (vertex_attrs[cache.local().v1].boundary_vertex &&
        vertex_attrs[cache.local().v2].boundary_vertex) {
        vertex_attrs[vid].boundary_vertex = true;
        vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;
        // compare collapse to which one would give lower energy
        vertex_attrs[vid].pos = vertex_attrs[cache.local().v1].pos;
        vertex_attrs[vid].t = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
        auto energy1 = get_one_ring_energy(edge_tuple).first;
        vertex_attrs[vid].pos = vertex_attrs[cache.local().v2].pos;
        vertex_attrs[vid].t = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
        auto energy2 = get_one_ring_energy(edge_tuple).first;
        p = energy1 < energy2 ? vertex_attrs[cache.local().v1].pos
                              : vertex_attrs[cache.local().v2].pos;
        t_parameter = energy1 < energy2 ? std::fmod(vertex_attrs[cache.local().v1].t, mod_length)
                                        : std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
    } else if (vertex_attrs[cache.local().v1].boundary_vertex) {
        p = vertex_attrs[cache.local().v1].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
    } else if (vertex_attrs[cache.local().v2].boundary_vertex) {
        p = vertex_attrs[cache.local().v2].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
    } else {
        assert(!vertex_attrs[cache.local().v1].boundary_vertex);
        assert(!vertex_attrs[cache.local().v2].boundary_vertex);
        p = (vertex_attrs[cache.local().v1].pos + vertex_attrs[cache.local().v2].pos) / 2.0;
        t_parameter = std::fmod(
            (vertex_attrs[cache.local().v1].t + vertex_attrs[cache.local().v2].t) / 2.0,
            mod_length);
        // !!! update t_parameter and check for periodicity + curvid !!!
    }
    if (vertex_attrs[cache.local().v1].fixed) {
        p = vertex_attrs[cache.local().v1].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
    } else if (vertex_attrs[cache.local().v2].fixed) {
        p = vertex_attrs[cache.local().v2].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
    } else {
        assert(!vertex_attrs[cache.local().v1].fixed);
        assert(!vertex_attrs[cache.local().v2].fixed);
        // this is the same case as both are not boundary
    }
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].t = t_parameter;
    vertex_attrs[vid].partition_id = cache.local().partition_id;
    vertex_attrs[vid].boundary_vertex =
        (vertex_attrs[cache.local().v1].boundary_vertex ||
         vertex_attrs[cache.local().v2].boundary_vertex);
    vertex_attrs[vid].fixed =
        (vertex_attrs[cache.local().v1].fixed || vertex_attrs[cache.local().v2].fixed);
    vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;
    // enforce heuristic
    if (length3d < (4. / 5. * mesh_parameters.m_target_l)) {
        return true;
    }
    // // check quality
    // auto tris = get_one_ring_tris_for_vertex(t);
    // for (auto tri : tris) {
    //     if (get_quality(tri) > cache.local().max_energy) return false;
    // }
    return false;
}
