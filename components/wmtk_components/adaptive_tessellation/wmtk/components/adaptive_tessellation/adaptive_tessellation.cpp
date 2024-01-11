#include "adaptive_tessellation.hpp"

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/HDF5Reader.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include <wmtk/components/adaptive_tessellation/operations/internal/ATOptions.hpp>

#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>

#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>

#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>

namespace wmtk::components {
using namespace operations;
using namespace function;
using namespace invariants;
namespace AT = wmtk::components;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& name,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        const std::filesystem::path data_dir = "";
        wmtk::io::ParaviewWriter writer(
            data_dir / (name + "_" + std::to_string(index)),
            "vertices",
            *mesh,
            true,
            true,
            true,
            false);
        mesh->serialize(writer);
        wmtk::io::ParaviewWriter writer3d(
            data_dir / ("accruacy_position_" + std::to_string(index)),
            "position",
            *mesh,
            true,
            true,
            true,
            false);
        mesh->serialize(writer3d);
    }
}
} // namespace

void adaptive_tessellation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    //////////////////////////////////
    // Load mesh from settings
    ATOptions options = j.get<ATOptions>();
    // const std::filesystem::path& file = options.input;
    std::shared_ptr<Mesh> mesh = cache.read_mesh(options.input);

    //////////////////////////////////
    // Storing edge lengths
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

    AT::operations::internal::ATData atdata(mesh, funcs);
    AT::operations::internal::ATOperations at_ops(atdata, options.target_edge_length);
    at_ops.set_energies();

    // std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
    //     std::make_shared<wmtk::function::TriangleAMIPS>(*mesh, atdata.uv_handle());

    // std::array<std::shared_ptr<image::Image>, 3> images = {
    //     {std::make_shared<image::Image>(500, 500),
    //      std::make_shared<image::Image>(500, 500),
    //      std::make_shared<image::Image>(500, 500)}};
    // auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    // auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    // std::function<double(double, double)> height_function =
    //     [](const double& u, [[maybe_unused]] const double& v) -> double {
    //     // return u * u + v * v;
    //     return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    // };
    // images[0]->set(u);
    // images[1]->set(v);
    // images[2]->set(height_function);
    // std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> texture =
    //     std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
    //         *mesh,
    //         atdata.uv_handle(),
    //         images);

    /*{ // face error update
        auto face_error_attribute =
            mesh->register_attribute<double>("face_error", PrimitiveType::Face, 1);
        auto face_error_accessor = mesh->create_accessor(face_error_attribute.as<double>());

        auto compute_face_error = [&evaluator](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
            assert(P.cols() == 3);
            assert(P.rows() == 2);
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);
            Eigen::Vector2<double> uv0 = P.col(0);
            Eigen::Vector2<double> uv1 = P.col(1);
            Eigen::Vector2<double> uv2 = P.col(2);
            Eigen::VectorXd error(1);
            error(0) = analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2);
            return error;
        };
        auto face_error_update =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                face_error_attribute,
                atdata.uv_handle(),
                compute_face_error);
        for (auto& f : mesh->get_all(PrimitiveType::Face)) {
            if (!mesh->is_ccw(f)) {
                f = mesh->switch_vertex(f);
            }
            const Eigen::Vector2d v0 = pt_accessor.vector_attribute(f);
            const Eigen::Vector2d v1 = pt_accessor.vector_attribute(mesh->switch_vertex(f));
            const Eigen::Vector2d v2 =
                pt_accessor.vector_attribute(mesh->switch_vertex(mesh->switch_edge(f)));
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);

            auto res = analytical_quadrature.get_error_one_triangle_exact(v0, v1, v2);
            face_error_accessor.scalar_attribute(f) = res;
        }
    }*/


    opt_logger().set_level(spdlog::level::level_enum::critical);


    // 1) wmtk::operations::EdgeSplit
    at_ops.AT_split_interior();


    // 2) EdgeCollapse
    at_ops.AT_collapse_interior(at_ops.m_sum_energy);

    // 3) EdgeSwap
    at_ops.AT_swap_interior(at_ops.m_sum_energy);

    // 4) Smoothing
    at_ops.AT_smooth_interior(at_ops.m_sum_energy);


    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    for (int64_t i = 0; i < options.passes; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        for (auto& op : at_ops.m_ops) pass_stats += scheduler.run_operation_on_all(*op);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);
        write(mesh, options.output, i + 1, options.intermediate_output);
    }
    // write(mesh, "no_operation", 0, options.intermediate_output);
    const std::filesystem::path data_dir = "";
    wmtk::io::ParaviewWriter
        writer(data_dir / ("output_pos"), "vert_pos", *mesh, true, true, true, false);
    mesh->serialize(writer);
}
} // namespace wmtk::components