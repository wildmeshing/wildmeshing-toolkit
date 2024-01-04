#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>

#include <wmtk/Scheduler.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/Operation.hpp>

namespace AT = wmtk::components::adaptive_tessellation;
namespace ATfunction = wmtk::components::adaptive_tessellation::function;
using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("test_smoothing")
{
    DEBUG_TriMesh uv_mesh = unit_squre();
    std::array<AT::image::Image, 3> images = {
        AT::image::Image(500, 500),
        AT::image::Image(500, 500),
        AT::image::Image(500, 500)};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    std::function<double(double, double)> height_function =
        [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0].set(u);
    images[1].set(v);
    images[2].set(height_function);

    AT::operations::internal::ATData atdata(std::make_shared<TriMesh>(uv_mesh), images);
    Scheduler scheduler;
    SchedulerStats pass_stats;

    AT::operations::internal::ATOperations at_ops(atdata, 0.1);
    at_ops.AT_split_interior();
    // at_ops.AT_split_boundary();
    for (int64_t i = 0; i < 6; ++i) {
        logger().info("Pass {}", i);
        for (auto& op : at_ops.m_ops) {
            pass_stats += scheduler.run_operation_on_all(*op);
        }
    }
    at_ops.m_ops.clear();
    at_ops.AT_smooth_interior();

    for (int64_t i = 0; i < 10; ++i) {
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