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
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>


#include <fstream>
#include <iostream>

namespace wmtk::components {
using namespace operations;
using namespace function;
using namespace invariants;
namespace AT = wmtk::components;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& uv_output,
    const std::string& xyz_output,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        const std::filesystem::path data_dir = "";
        wmtk::io::ParaviewWriter writer(
            data_dir / (uv_output + "_" + std::to_string(index)),
            "vertices",
            *mesh,
            true,
            true,
            true,
            false);
        mesh->serialize(writer);
        wmtk::io::ParaviewWriter writer3d(
            data_dir / (xyz_output + "_" + std::to_string(index)),
            "position",
            *mesh,
            true,
            true,
            true,
            false);
        mesh->serialize(writer3d);
    }
}
void write_face_attr(
    const std::shared_ptr<Mesh>& mesh,
    const Accessor<double>& face_error_accessor,
    nlohmann::ordered_json& jsonData,
    const int64_t index,
    const std::string& filename)
{
    // Create an array under the key "data"
    jsonData["itr_" + std::to_string(index)] = nlohmann::json::array();
    for (auto& f : mesh->get_all(PrimitiveType::Face)) {
        double res = face_error_accessor.scalar_attribute(f);
        jsonData["itr_" + std::to_string(index)].push_back(res);
    }
    // Open the file in append mode
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        outputFile << jsonData.dump(4);
        outputFile.close();
        std::cout << "JSON data written to " << filename << std::endl;
    } else {
        throw std::runtime_error("Unable to open face error json file");
    }
}


void _debug_texture_integral(
    std::shared_ptr<Mesh> mesh,
    wmtk::attribute::MeshAttributeHandle m_uv_handle,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& image_evaluator,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& func_evaluator)
{
    Accessor<double> m_uv_accessor = mesh->create_accessor(m_uv_handle.as<double>());
    wmtk::attribute::MeshAttributeHandle image_res_handle =
        mesh->register_attribute<double>("image_res", PrimitiveType::Face, 1);
    Accessor<double> image_res_accessor = mesh->create_accessor(image_res_handle.as<double>());
    wmtk::attribute::MeshAttributeHandle func_res_handle =
        mesh->register_attribute<double>("func_res", PrimitiveType::Face, 1);
    Accessor<double> func_res_accessor = mesh->create_accessor(func_res_handle.as<double>());
    for (auto& f : mesh->get_all(PrimitiveType::Face)) {
        if (!mesh->is_ccw(f)) {
            f = mesh->switch_vertex(f);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 = m_uv_accessor.vector_attribute(mesh->switch_vertex(f));
        const Eigen::Vector2d uv2 =
            m_uv_accessor.vector_attribute(mesh->switch_vertex(mesh->switch_edge(f)));

        wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature
            analytical_quadrature(func_evaluator);
        double func_res = analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2);
        func_res_accessor.scalar_attribute(f) = func_res;

        wmtk::components::function::utils::TextureIntegral texture_integral(image_evaluator);
        double image_res = texture_integral.get_error_one_triangle_exact(uv0, uv1, uv2);
        image_res_accessor.scalar_attribute(f) = image_res;
        std::cout << "func_res: " << func_res << std::endl;
        std::cout << "image_res: " << image_res << std::endl;
    }
    write(mesh, "diff_sampling_debug_og_l4", "at_sampling_flat__xyz_output", 0, 1);
}
} // namespace

void adaptive_tessellation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    //////////////////////////////////
    // Load mesh from settings
    ATOptions options = j.get<ATOptions>();
    std::cout << "at_ops.barrier_weight: " << options.barrier_weight << std::endl;
    std::cout << "options.barrier_triangle_area: " << options.barrier_triangle_area << std::endl;
    std::cout << "options.quadrature_weight: " << options.quadrature_weight << std::endl;
    std::cout << "options.amips_weight: " << options.amips_weight << std::endl;
    std::cout << "options.passes: " << options.passes << std::endl;
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
         //  std::make_shared<image::SamplingAnalyticFunction>(
         //      image::SamplingAnalyticFunction_FunctionType::Periodic,
         //      2,
         //      2,
         //      1.)
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Gaussian,
             0.5,
             0.5,
             1.)

        }};

    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500)}};

    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return exp(-(pow(u - 0.5, 2) + pow(v - 0.5, 2)) / (2 * 0.1 * 0.1));
        // return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u);
    images[1]->set(v);
    images[2]->set(height_function);

    // AT::operations::internal::ATData atdata(mesh, funcs);
    AT::operations::internal::ATData atdata(mesh, images, AT::image::SAMPLING_METHOD::Bilinear);
    // wmtk::components::function::utils::ThreeChannelPositionMapEvaluator image_evaluator(
    //     images,
    //     image::SAMPLING_METHOD::Bicubic,
    //     image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT);
    // wmtk::components::function::utils::ThreeChannelPositionMapEvaluator func_evaluator(funcs);
    // atdata._debug_sampling(image_evaluator, func_evaluator);

    // _debug_texture_integral(mesh, atdata.uv_handle(), image_evaluator, func_evaluator);
    // exit(0);

    AT::operations::internal::ATOperations at_ops(
        atdata,
        options.target_edge_length,
        options.barrier_weight,
        options.barrier_triangle_area,
        options.quadrature_weight,
        options.amips_weight,
        options.area_weighted_amips);


    at_ops.set_energies();
    nlohmann::ordered_json FaceErrorJson_sum;
    nlohmann::ordered_json FaceErrorJson_amips;
    write(mesh, options.uv_output, options.xyz_output, 0, options.intermediate_output);
    write_face_attr(
        mesh,
        at_ops.m_sum_error_accessor,
        FaceErrorJson_sum,
        0,
        options.uv_output + "_face_error.json");
    write_face_attr(
        mesh,
        at_ops.m_amips_error_accessor,
        FaceErrorJson_amips,
        0,
        options.uv_output + "_amips_error.json");
    opt_logger().set_level(spdlog::level::level_enum::critical);


    // 1) wmtk::operations::EdgeSplit
    // at_ops.AT_split_interior(at_ops.m_high_error_edges_first, at_ops.m_sum_energy);


    // 3) EdgeSwap
    // at_ops.AT_swap_interior(at_ops.m_valence_improvement, at_ops.m_sum_energy);

    // 4) Smoothing
    at_ops.AT_smooth_interior(at_ops.m_sum_energy);


    // nlohmann::ordered_json FaceErrorJson;
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

        write_face_attr(
            mesh,
            at_ops.m_sum_error_accessor,
            FaceErrorJson_sum,
            i + 1,
            options.uv_output + "_face_error.json");
        write_face_attr(
            mesh,
            at_ops.m_amips_error_accessor,
            FaceErrorJson_amips,
            i + 1,
            options.uv_output + "_amips_error.json");

        write(mesh, options.uv_output, options.xyz_output, i + 1, options.intermediate_output);
    }

    // write(mesh, "no_operation", 0, options.intermediate_output);
}

} // namespace wmtk::components