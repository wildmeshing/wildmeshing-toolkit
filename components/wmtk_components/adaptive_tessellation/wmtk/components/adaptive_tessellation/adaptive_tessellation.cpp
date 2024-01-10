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
            data_dir / ("3d_" + std::to_string(index)),
            "vert_pos",
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
    // auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(*mesh);
    // collapse->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    // collapse->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    // collapse->add_invariant(
    //     std::make_shared<SimplexInversionInvariant>(*mesh, atdata.uv_handle().as<double>()));
    // collapse->add_invariant(
    //     std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), atdata.m_amips_energy));
    // collapse->add_invariant(std::make_shared<TodoSmallerInvariant>(
    //     *mesh,
    //     atdata.m_3d_edge_length_handle.as<double>(),
    //     4.0 / 5.0 * at_ops.m_target_edge_length));
    // collapse->set_priority(short_edges_first);

    // auto clps_strat = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
    //     atdata.uv_handle());
    // clps_strat->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    // clps_strat->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);

    // collapse->set_new_attribute_strategy(atdata.uv_handle(), clps_strat);
    // collapse->set_new_attribute_strategy(atdata.m_3d_edge_length_handle);

    // collapse->add_transfer_strategy(edge_length_update);

    // auto clps_strat2 = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
    //     vert_pos_attribute);
    // clps_strat2->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    // clps_strat2->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);

    // collapse->set_new_attribute_strategy(vert_pos_attribute, clps_strat2);
    // // collapse->set_new_attribute_strategy(face_error_attribute);

    // collapse->add_transfer_strategy(vert_position_update);
    // // collapse->add_transfer_strategy(face_error_update);
    // ops.emplace_back(collapse);


    // 3) TriEdgeSwap
    // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    //     auto swap = std::make_shared<TriEdgeSwap>(*mesh);
    //     swap->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    //     swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    //     swap->add_invariant(
    //         std::make_shared<SimplexInversionInvariant>(*mesh, atdata.uv_handle().as<double>()));
    //     swap->add_invariant(
    //         std::make_shared<FunctionInvariant>(mesh->top_simplex_type(),
    //         atdata.m_amips_energy));
    //     swap->set_priority(long_edges_first);

    //     swap->collapse().set_new_attribute_strategy(vert_pos_attribute);
    //     swap->split().set_new_attribute_strategy(vert_pos_attribute);
    //     swap->collapse().set_new_attribute_strategy(atdata.m_3d_edge_length_handle);
    //     swap->split().set_new_attribute_strategy(atdata.m_3d_edge_length_handle);

    //     swap->split().set_new_attribute_strategy(atdata.uv_handle());
    //     swap->collapse().set_new_attribute_strategy(
    //         atdata.uv_handle(),
    //         wmtk::operations::CollapseBasicStrategy::CopyOther);
    //     swap->split().set_new_attribute_strategy(vert_pos_attribute);
    //     swap->collapse().set_new_attribute_strategy(
    //         vert_pos_attribute,
    //         wmtk::operations::CollapseBasicStrategy::CopyOther);

    //     swap->add_transfer_strategy(edge_length_update);

    //     ops.push_back(swap);
    // } else // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    // {
    //     throw std::runtime_error("unsupported");
    // }

    // 4) Smoothing
    at_ops.AT_smooth_interior(atdata.m_amips_energy);


    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    for (int64_t i = 0; i < 20; ++i) {
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