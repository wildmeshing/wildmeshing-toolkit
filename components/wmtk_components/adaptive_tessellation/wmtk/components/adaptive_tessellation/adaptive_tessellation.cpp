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
#include <wmtk/io/ObjWriter.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include "ATOptions.hpp"

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

#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATScheduler.hpp>
#include <wmtk/components/adaptive_tessellation/operations/utils/tag_todo_edges.hpp>

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

#include <wmtk/components/adaptive_tessellation/image/utils/load_image_exr.hpp>

#include <fstream>
#include <iostream>
#include <wmtk/components/adaptive_tessellation/operations/internal/AT_debug.cpp>

#include <wmtk/multimesh/consolidate.hpp>
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
    const int64_t index)
{
    const std::filesystem::path data_dir = "";
    wmtk::io::ParaviewWriter writer(
        data_dir / (uv_output + "_" + std::to_string(index)),
        "vertices",
        *uv_mesh,
        false,
        true,
        true,
        false);
    logger().info("saving uv mesh on {}", data_dir / (uv_output + "_" + std::to_string(index)));
    uv_mesh->serialize(writer);
    wmtk::io::ParaviewWriter writer3d(
        data_dir / (xyz_output + "_" + std::to_string(index)),
        "positions",
        *uv_mesh,
        false,
        false,
        true,
        false);
    logger().info("saving xyz mesh on {}", data_dir / (xyz_output + "_" + std::to_string(index)));
    uv_mesh->serialize(writer3d);
    wmtk::io::ObjWriter writer3d_obj(
        data_dir / (xyz_output + "_" + std::to_string(index)),
        "positions",
        *uv_mesh);
    uv_mesh->serialize(writer3d_obj);
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
    AT::operations::internal::ATData atdata;
    if (options.analytical_function) {
        wmtk::logger().critical("///// using gaussian displacement /////");
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
             std::make_shared<image::SamplingAnalyticFunction>(
                 image::SamplingAnalyticFunction_FunctionType::Gaussian,
                 0.5,
                 0.5,
                 1.)}};
        atdata = AT::operations::internal::ATData(position_mesh_ptr, uv_mesh_ptr, funcs);
    } else if (options.terrain_texture) {
        auto
            [w_h,
             h_h,
             index_red_h,
             index_green_h,
             index_blue_h,
             buffer_r_h,
             buffer_g_h,
             buffer_b_h] =
                wmtk::components::image::load_image_exr_split_3channels(options.height_path);
        std::array<std::shared_ptr<image::Image>, 3> images = {
            {std::make_shared<image::Image>(w_h, h_h),
             std::make_shared<image::Image>(w_h, h_h),
             std::make_shared<image::Image>(w_h, h_h)}};

        auto u_func = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
        auto v_func = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
        auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
            return exp(-(pow(u - 0.5, 2) + pow(v - 0.5, 2)) / (2 * 0.1 * 0.1));
        };
        images[0]->set(u_func);
        images[1]->set(v_func);
        // images[2]->set(height_function);

        images[2] = std::make_shared<wmtk::components::image::Image>(
            wmtk::components::image::buffer_to_image(buffer_r_h, w_h, h_h));
        logger().warn("height image loaded size {}", w_h);
        atdata = AT::operations::internal::ATData(
            position_mesh_ptr,
            uv_mesh_ptr,
            images,
            options.max_distance);
    } else {
        atdata = AT::operations::internal::ATData(
            position_mesh_ptr,
            uv_mesh_ptr,
            options.position_path,
            options.normal_path,
            options.height_path,
            options.max_distance,
            options.min_height,
            options.max_height);
    }

    AT::operations::internal::ATOperations at_ops(
        atdata,
        options.target_distance,
        options.target_triangle_number);

    /////////////////////////////////////////////////
    // mesh refinement to decrease distance error
    {
        nlohmann::ordered_json FaceErrorJson_distance;
        nlohmann::ordered_json FaceErrorJson_amips;
        write(position_mesh_ptr, uv_mesh_ptr, options.uv_output, options.xyz_output, 0);

        opt_logger().set_level(spdlog::level::level_enum::critical);


        int64_t rgb_split_index = at_ops.AT_rgb_split();
        int64_t rgb_swap_index = at_ops.AT_rgb_swap();
        ATScheduler scheduler;
        int cnt = 0;
        while (true) {
            int64_t todo_edge_cnt =
                wmtk::components::operations::utils::tag_longest_edge_of_all_faces(
                    atdata.uv_mesh_ptr(),
                    at_ops.m_edge_todo_accessor,
                    at_ops.m_distance_error_accessor,
                    at_ops.m_curved_edge_length_accessor,
                    options.target_distance);

            if (todo_edge_cnt == 0) {
                break;
            }
            multimesh::consolidate(*atdata.uv_mesh_ptr());
            if (options.target_triangle_number > 0 &&
                atdata.uv_mesh_ptr()->capacity(PrimitiveType::Triangle) >=
                    options.target_triangle_number) {
                break;
            }
            logger().critical("outter to do is not zero and triangle number is not enough");
            while (true) {
                int64_t inner_i = 1;
                while (true) {
                    const auto stats =
                        scheduler.run_operation_on_all(*at_ops.m_ops[rgb_split_index]);
                    logger().warn("finished split");
                    if (stats.number_of_successful_operations() == 0) {
                        break;
                    }
                }
                multimesh::consolidate(*atdata.uv_mesh_ptr());
                if (options.target_triangle_number > 0 &&
                    atdata.uv_mesh_ptr()->capacity(PrimitiveType::Triangle) >=
                        options.target_triangle_number) {
                    break;
                }
                while (true) {
                    const auto stats =
                        scheduler.run_operation_on_all(*at_ops.m_ops[rgb_swap_index]);
                    logger().warn("finished swap");
                    multimesh::consolidate(*atdata.uv_mesh_ptr());
                    if (stats.number_of_successful_operations() == 0) {
                        break;
                    }
                }
                int64_t inner_todo_edge_cnt = 0;
                for (auto& e : atdata.uv_mesh_ptr()->get_all(wmtk::PrimitiveType::Edge)) {
                    if (at_ops.m_edge_todo_accessor.scalar_attribute(e) == 1) {
                        wmtk::components::operations::utils::tag_secondary_split_edges(
                            atdata.uv_mesh_ptr(),
                            at_ops.m_face_rgb_state_accessor,
                            at_ops.m_edge_rgb_state_accessor,
                            at_ops.m_edge_todo_accessor,
                            e);
                        inner_todo_edge_cnt++;
                    }
                }
                if (inner_todo_edge_cnt == 0) {
                    break;
                }
                inner_i++;
                logger().critical("inner to do is not zero");
            }
            cnt++;
        }
        multimesh::consolidate(*atdata.uv_mesh_ptr());
        write(
            atdata.uv_mesh_ptr(),
            atdata.uv_mesh_ptr(),
            options.uv_output,
            options.xyz_output,
            cnt);
    }

    at_ops.m_ops.clear();

    std::vector<attribute::MeshAttributeHandle> keeps;
    keeps.emplace_back(atdata.m_uvmesh_xyz_handle);
    atdata.uv_mesh_ptr()->clear_attributes(keeps);

    cache.write_mesh(*atdata.uv_mesh_ptr(), options.xyz_output);
}

} // namespace wmtk::components