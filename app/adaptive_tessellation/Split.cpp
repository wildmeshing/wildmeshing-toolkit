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

// every edge is split if it is longer than 4/5 L

auto split_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

// split edge accuracy error

// optimized to find the new vertex position

void AdaptiveTessellation::split_all_edges()
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
        executor.priority = [&](auto& m, auto _, auto& e) {
            if (m.mesh_parameters.m_accuracy) {
                Eigen::Matrix<double, 2, 1> pos0 = m.vertex_attrs[e.vid(m)].pos;
                Eigen::Matrix<double, 2, 1> pos1 = m.vertex_attrs[e.switch_vertex(m).vid(m)].pos;
                // Eigen::Matrix<double, 2, 1> posnew = (pos0 + pos1) * 0.5;
                // auto error1 = m.mesh_parameters.m_displacement->get_error_per_edge(pos0, posnew);
                // auto error2 = m.mesh_parameters.m_displacement->get_error_per_edge(posnew, pos1);
                auto length = m.mesh_parameters.m_displacement->get_error_per_edge(pos0, pos1);
                return length;
            }
            return m.mesh_parameters.m_get_length(e.vid(m), e.switch_vertex(m).vid(m));
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.mesh_parameters.m_get_length(tup.vid(m), tup.switch_vertex(m).vid(m));
            // if (m.mesh_parameters.m_accuracy) {
            //     Eigen::Matrix<double, 2, 1> pos0 = m.vertex_attrs[tup.vid(m)].pos;
            //     Eigen::Matrix<double, 2, 1> pos1 =
            //     m.vertex_attrs[tup.switch_vertex(m).vid(m)].pos; Eigen::Matrix<double, 2, 1>
            //     posnew = (pos0 + pos1) * 0.5; auto error1 =
            //     m.mesh_parameters.m_displacement->get_error_per_edge(pos0, posnew); auto error2 =
            //     m.mesh_parameters.m_displacement->get_error_per_edge(posnew, pos1); length -=
            //     (error1 + error2);
            // }
            if (abs(length - weight) > 1e-10) return false;
            if (m.mesh_parameters.m_accuracy) {
                if (length < m.mesh_parameters.m_accuracy_threshold) return false;
            } else if (length < 4. / 3. * m.mesh_parameters.m_target_l)
                return false;
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
bool AdaptiveTessellation::split_edge_before(const Tuple& edge_tuple)
{
    static std::atomic_int cnt = 0;
    // if (cnt % 50 == 0)
    write_vtk(mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.vtu", cnt));

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
    cnt++;
    return true;
}
bool AdaptiveTessellation::split_edge_after(const Tuple& edge_tuple)
{
    // adding heuristic decision. If length2 > 4. / 3. * 4. / 3. * m.m_target_l * m.m_target_l always split
    // transform edge length with displacement

    double length3d = mesh_parameters.m_get_length(cache.local().v1, cache.local().v2);
    if (length3d < 0.) return false;
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
        vertex_attrs[vid].t = mesh_parameters.m_boundary.uv_to_t(vertex_attrs[vid].pos);
    }
    // enforce length check
    if (mesh_parameters.m_accuracy) {
        // auto after_operation_error = length3d - get_accuracy_error(cache.local().v1, vid) -
        //                              get_accuracy_error(cache.local().v2, vid);
        // // operation happens when error difference is higher than user determined value
        // // if (after_operation_error > mesh_parameters.m_accuracy_threshold) return true;
        // // [Change To] if after operation error is bigger than before, reject operation
        // // o.w. accept
        // if (after_operation_error >= -1e-4) return true;
        return true;
    } else if (length3d > (4. / 3. * mesh_parameters.m_target_l)) {
        return true;
    }
    return false;
}
