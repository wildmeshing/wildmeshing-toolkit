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

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

#include <fstream>
#include <iostream>
#include <wmtk/components/adaptive_tessellation/operations/internal/AT_debug.cpp>

namespace wmtk::components {
using namespace operations;
using namespace function;
using namespace invariants;
namespace AT = wmtk::components;

namespace {
void write(
    const std::shared_ptr<Mesh>& position_mesh,
    const std::shared_ptr<Mesh>& uv_mesh,
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
            *uv_mesh,
            true,
            true,
            true,
            false);
        uv_mesh->serialize(writer);
        wmtk::io::ParaviewWriter writer3d(
            data_dir / (xyz_output + "_" + std::to_string(index)),
            "positions",
            *uv_mesh,
            true,
            true,
            true,
            false);
        uv_mesh->serialize(writer3d);
    }
}
void write_face_attr(
    const std::shared_ptr<Mesh>& mesh,
    wmtk::attribute::Accessor<double>& face_error_accessor,
    nlohmann::ordered_json& jsonData,
    const int64_t index,
    const std::string& filename)
{
    // Create an array under the key "data"
    jsonData["itr_" + std::to_string(index)] = nlohmann::json::array();
    for (auto& f : mesh->get_all(PrimitiveType::Triangle)) {
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

} // namespace

void adaptive_tessellation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    //////////////////////////////////
    // Load mesh from settings
    ATOptions options = j.get<ATOptions>();

    std::shared_ptr<Mesh> position_mesh_ptr = cache.read_mesh(options.parent);
    std::shared_ptr<Mesh> uv_mesh_ptr = cache.read_mesh(options.child);

    //////////////////////////////////
    // Storing edge lengths
    wmtk::logger().critical("///// using terrain displacement /////");
    std::array<std::shared_ptr<image::Sampling>, 3> funcs = {
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
         // std::make_shared<image::SamplingAnalyticFunction>(
         //     image::SamplingAnalyticFunction_FunctionType::Linear,
         //     0,
         //     0,
         //     1.)
         // std::make_shared<image::SamplingAnalyticFunction>(
         //     image::SamplingAnalyticFunction_FunctionType::Periodic,
         //     2,
         //     2,
         //     1.)
         // std::make_shared<image::SamplingAnalyticFunction>(
         //     image::SamplingAnalyticFunction_FunctionType::Gaussian,
         //     0.5,
         //     0.5,
         //     1.)
         std::make_shared<image::ProceduralFunction>(image::ProceduralFunctionType::Terrain)

        }};

    // std::make_shared<image::ProceduralFunction>(image::ProceduralFunctionType::Terrain)
    //     ->convert_to_exr(512, 512);
    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500)}};

    auto u_func = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v_func = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return exp(-(pow(u - 0.5, 2) + pow(v - 0.5, 2)) / (2 * 0.1 * 0.1));
        // return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u_func);
    images[1]->set(v_func);
    images[2]->set(height_function);


    // AT::operations::internal::ATData atdata(
    //     position_mesh_ptr,
    //     uv_mesh_ptr,
    //     options.position_path,
    //     options.normal_path,
    //     options.height_path);
    // AT::operations::internal::ATData atdata(position_mesh_ptr, uv_mesh_ptr, images);
    AT::operations::internal::ATData atdata(position_mesh_ptr, uv_mesh_ptr, funcs);

    AT::operations::internal::ATOperations at_ops(
        atdata,
        options.target_edge_length,
        options.barrier_weight,
        options.barrier_triangle_area,
        options.quadrature_weight,
        options.amips_weight,
        options.area_weighted_amips);


    at_ops.set_energies();
    nlohmann::ordered_json FaceErrorJson_distance;
    nlohmann::ordered_json FaceErrorJson_amips;
    write(
        position_mesh_ptr,
        uv_mesh_ptr,
        options.uv_output,
        options.xyz_output,
        0,
        options.intermediate_output);

    opt_logger().set_level(spdlog::level::level_enum::critical);


    // 1.5) FaceSplit
    // at_ops.AT_face_split(at_ops.m_high_distance_faces_first, at_ops.m_distance_nondiff_energy);
    // 1) wmtk::operations::EdgeSplit
    // at_ops.AT_edge_split(at_ops.m_high_distance_edges_first, at_ops.m_distance_nondiff_energy);
    // at_ops.AT_boundary_edge_split(
    //     at_ops.m_high_distance_edges_first,
    //     at_ops.m_distance_nondiff_energy);
    // 3) EdgeSwap
    // at_ops.AT_swap_interior(at_ops.m_high_amips_edges_first, at_ops.m_distance_nondiff_energy);

    // 4) Smoothing
    // at_ops.AT_smooth_interior(at_ops.m_distance_energy);

    /// split on amips error

    at_ops.AT_edge_split(at_ops.m_high_distance_faces_first, at_ops.m_3d_amips_energy);
    Scheduler scheduler;
    bool success = true;
    int64_t i = 0;
    do {
        i++;
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;

        success =
            at_ops.single_split_execution(*at_ops.m_ops[0], at_ops.m_triangle_distance_edge_length);
        // only output the log if it is 100 modulo
        if (i % 100 == 0) {
            logger().info("Executed {} op Succeed? {}", i, success);
        }

    } while (success);
    write(
        uv_mesh_ptr,
        uv_mesh_ptr,
        options.uv_output,
        options.xyz_output,
        i + 1,
        options.intermediate_output);
    write_face_attr(
        uv_mesh_ptr,
        at_ops.m_distance_error_accessor,
        FaceErrorJson_distance,
        i + 1,
        options.uv_output + "_distance_error.json");
    // at_ops.AT_swap_interior(at_ops.m_high_amips_edges_first, at_ops.m_3d_amips_energy);
    at_ops.m_ops.clear();
    at_ops.AT_smooth_interior(at_ops.m_2d_amips_energy);


    // nlohmann::ordered_json FaceErrorJson;
    //////////////////////////////////
    // Running all ops in order n times
    // Scheduler scheduler;
    // opt_logger().set_level(spdlog::level::level_enum::debug);
    for (int64_t i = 0; i < 0; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        for (auto& op : at_ops.m_ops) pass_stats += scheduler.run_operation_on_all(*op);

        // cache.write_mesh(*uv_mesh_ptr, "bumpyDice_debug_" + std::to_string(i));
        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        // write_face_attr(
        //     uv_mesh_ptr,
        //     at_ops.m_distance_error_accessor,
        //     FaceErrorJson_sum,
        //     i + 1,
        //     options.uv_output + "_distance_error.json");
        write_face_attr(
            uv_mesh_ptr,
            at_ops.m_amips_error_accessor,
            FaceErrorJson_amips,
            i + 1,
            options.uv_output + "_2d_amips_error.json");

        write(
            uv_mesh_ptr,
            uv_mesh_ptr,
            options.uv_output,
            options.xyz_output,
            i + 1,
            options.intermediate_output);
    }

    // write(mesh, "no_operation", 0, options.intermediate_output);
}

} // namespace wmtk::components