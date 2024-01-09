#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>

#include <wmtk/Scheduler.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include "utils/unit_square_test_case.hpp"

namespace AT = wmtk::components;
namespace ATfunction = wmtk::components::function;
using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("test_at")
{
    DEBUG_TriMesh uv_mesh = unit_squre();
    DEBUG_TriMesh position_mesh = quad();
    std::array<std::shared_ptr<AT::image::Image>, 3> images = {
        std::make_shared<AT::image::Image>(500, 500),
        std::make_shared<AT::image::Image>(500, 500),
        std::make_shared<AT::image::Image>(500, 500)};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    std::function<double(double, double)> height_function =
        [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u);
    images[1]->set(v);
    images[2]->set(height_function);

    std::pair<std::vector<std::shared_ptr<Mesh>>, std::map<Mesh*, Mesh*>> edge_mesh_ret =
        unit_square_example(uv_mesh, position_mesh, images);

    AT::operations::internal::ATData atdata(
        std::make_shared<TriMesh>(uv_mesh),
        std::make_shared<TriMesh>(position_mesh),
        edge_mesh_ret.first,
        edge_mesh_ret.second,
        images);
    Scheduler scheduler;
    SchedulerStats pass_stats;

    AT::operations::internal::ATOperations at_ops(atdata, 0.1);
    at_ops.AT_split_interior();
    // at_ops.AT_split_boundary();
    for (int64_t i = 0; i < 2; ++i) {
        logger().info("Pass {}", i);
        for (auto& op : at_ops.m_ops) {
            pass_stats += scheduler.run_operation_on_all(*op);
        }
    }
    const std::filesystem::path data_dir = "";
    wmtk::io::ParaviewWriter
        writer(data_dir / ("split_result"), "vertices", uv_mesh, true, true, true, false);
    uv_mesh.serialize(writer);
    logger().info(
        "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
        pass_stats.number_of_performed_operations(),
        pass_stats.number_of_successful_operations(),
        pass_stats.number_of_failed_operations(),
        pass_stats.collecting_time,
        pass_stats.sorting_time,
        pass_stats.executing_time);

    // wmtk::components::AT_smooth_interior(atdata, ops);
}