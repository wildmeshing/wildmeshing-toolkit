#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <lagrange/utils/fpe.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <limits>
#include <optional>
using namespace adaptive_tessellation;
using namespace wmtk;

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

// list the triangle verteics in order
// return cost 0.for colinear or flipped triangle after swap (to save quadrature computation)
auto swap_accuracy_cost = [](auto& m, const TriMesh::Tuple& e, const double valence_cost) {
    double e_before = 0.0;
    auto e_after = e_before;
    if ((e.switch_face(m)).has_value()) {
        lagrange::enable_fpe();
        if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::ACCURACY) {
            e_before = m.mesh_parameters.m_get_length(e);
            auto t4 = (((e.switch_face(m)).value()).switch_edge(m)).switch_vertex(m);
            auto t3 = (e.switch_edge(m)).switch_vertex(m);
            auto t3_pos = m.vertex_attrs[t3.vid(m)].pos;
            auto t4_pos = m.vertex_attrs[t4.vid(m)].pos;
            e_after = m.mesh_parameters.m_displacement->get_error_per_edge(t3_pos, t4_pos);
            // positive if error decreases
            return e_before - e_after;
        } else
        // (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY)
        {
            auto other_face = e.switch_face(m).value();
            auto other_vid = other_face.switch_edge(m).switch_vertex(m).vid(m);
            // get oriented vids
            auto vids1 = m.oriented_tri_vids(e);
            auto vids2 = m.oriented_tri_vids(other_face);

            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle1;
            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle2;

            // replace the vids with the swapped vids
            for (auto i = 0; i < 3; i++) {
                if (vids1[i] == e.switch_vertex(m).vid(m)) vids1[i] = other_vid;
                if (vids2[i] == e.vid(m)) vids2[i] = e.switch_edge(m).switch_vertex(m).vid(m);
            }
            for (auto i = 0; i < 3; i++) {
                triangle1.row(i) = m.vertex_attrs[vids1[i]].pos;
                triangle2.row(i) = m.vertex_attrs[vids2[i]].pos;
            }

            if (polygon_signed_area(triangle1) <= 0 || polygon_signed_area(triangle2) <= 0) {
                return -std::numeric_limits<double>::infinity();
            } else {
                e_after = m.mesh_parameters.m_displacement->get_error_per_triangle(triangle1);
                e_after += m.mesh_parameters.m_displacement->get_error_per_triangle(triangle2);
            }
            if (valence_cost <= 0) return -std::numeric_limits<double>::infinity();
            if (e_after > m.mesh_parameters.m_accuracy_threshold)
                // the accuracy exceeds global bond
                return -std::numeric_limits<double>::infinity();
            return valence_cost * e_after;
        }
        return -std::numeric_limits<double>::infinity();
    }
};

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
        executor.renew_neighbor_tuples = swap_renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            if (m.is_boundary_edge(e))
                return -std::numeric_limits<double>::infinity(); // boundary edge shouldn't swap
            double valence_cost = swap_cost(m, e);
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                auto current_error = swap_accuracy_cost(m, e, valence_cost);
                return current_error;
            }
            return valence_cost;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            if (weight == -std::numeric_limits<double>::infinity()) return false;
            double current_cost = 0.;
            current_cost = swap_cost(m, tup);
            if (current_cost < 0.) return false;
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                current_cost = swap_accuracy_cost(m, tup, current_cost);
            }
            if (!is_close(current_cost, weight)) return false;
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
    if (!TriMesh::swap_edge_before(t)) return false;

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
