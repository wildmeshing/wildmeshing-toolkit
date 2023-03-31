#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"

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
            auto error = m.mesh_parameters.m_get_length(e);
            wmtk::logger().info("error {}", error);
            return error;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            double length = 0.;
            double error1 = 0.;
            double error2 = 0.;
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                error1 = m.get_area_accuracy_error_per_face(tup);
                if (tup.switch_face(m).has_value()) {
                    error2 = m.get_area_accuracy_error_per_face(tup.switch_face(m).value());
                } else
                    error2 = error1;
                length = (error1 + error2) * m.get_length2d(tup);
            } else
                length = m.mesh_parameters.m_get_length(tup);
            // check if out of date
            if (!is_close(length, weight)) return false;
            // check if meet operating threshold
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::ACCURACY) {
                if (length < m.mesh_parameters.m_accuracy_threshold) return false;
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                if (error1 < m.mesh_parameters.m_accuracy_threshold &&
                    error2 < m.mesh_parameters.m_accuracy_threshold)
                    return false;
            } else if (length < 4. / 3. * m.mesh_parameters.m_quality_threshold)
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
    if (!TriMesh::split_edge_before(edge_tuple)) return false;
    // if (cnt % 3 == 0) {
    write_vtk(mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.vtu", cnt));
    write_perface_vtk(mesh_parameters.m_output_folder + fmt::format("/tri_{:04d}.vtu", cnt));
    // }

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
    static std::atomic_int success_cnt = 0;
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
    wmtk::logger().info("split success {}", success_cnt);
    success_cnt++;
    return true;
}
