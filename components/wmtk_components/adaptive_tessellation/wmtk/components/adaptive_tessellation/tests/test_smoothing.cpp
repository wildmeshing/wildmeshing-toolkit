#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>

#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/Operation.hpp>

#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>

namespace AT = wmtk::components;
namespace ATfunction = wmtk::components::function;
using namespace wmtk;
using namespace wmtk::tests;
using json = nlohmann::json;
const std::filesystem::path data_dir = WMTK_DATA_DIR;
TEST_CASE("test_smoothing")
{
    std::shared_ptr<Mesh> uv_mesh_ptr = read_mesh(data_dir / "quad.msh", true);


    std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs = {
        {std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             1,
             0,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             1,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Periodic,
             2,
             2,
             1.)}
        //  std::make_shared<image::SamplingAnalyticFunction>(
        //      image::SamplingAnalyticFunction_FunctionType::Linear,
        //      0,
        //      0,
        //      0.)}

    };

    AT::operations::internal::ATData atdata(uv_mesh_ptr, funcs);
    Scheduler scheduler;
    SchedulerStats pass_stats;

    AT::operations::internal::ATOperations at_ops(atdata, 0.1);
    // at_ops.AT_split_interior();
    // at_ops.AT_split_boundary();
    // for (int64_t i = 0; i < 6; ++i) {
    //     logger().info("Pass {}", i);
    //     for (auto& op : at_ops.m_ops) {
    //         pass_stats += scheduler.run_operation_on_all(*op);
    //     }
    // }
    // at_ops.m_ops.clear();
    at_ops.AT_smooth_analytical(
        uv_mesh_ptr,
        uv_mesh_ptr->attribute_handle<double>("vertices", Primitive::Vertex));

    for (int64_t i = 0; i < 10; ++i) {
        logger().info("Pass {}", i);
        for (auto& op : at_ops.m_ops) {
            pass_stats += scheduler.run_operation_on_all(*op);
        }
    }

    const std::filesystem::path data_dir = "";
    wmtk::io::ParaviewWriter
        writer(data_dir / ("smooth_test"), "vertices", uv_mesh, true, true, true, false);
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