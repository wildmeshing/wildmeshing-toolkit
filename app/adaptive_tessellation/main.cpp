#include <AdaptiveTessellation.h>
#include <igl/Timer.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <lagrange/IndexedAttribute.h>
#include <lagrange/attribute_names.h>
#include <lagrange/foreach_attribute.h>
#include <lagrange/io/load_mesh.h>
#include <lagrange/triangulate_polygonal_facets.h>
#include <lagrange/utils/fpe.h>
#include <lagrange/utils/timing.h>
#include <lagrange/views.h>
#include <remeshing/UniformRemeshing.h>
#include <spdlog/common.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/Energy2d.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <CLI/CLI.hpp>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <regex>
#include <tracy/Tracy.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "AdaptiveTessellation.h"
#include "Parameters.h"

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;
using json = nlohmann::json;

using namespace wmtk;
using namespace adaptive_tessellation;
using namespace lagrange;
int main(int argc, char** argv)
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    ZoneScopedN("adaptive_tessellation_main");
    lagrange::enable_fpe();
    CLI::App app{argv[0]};
    std::string input_json;
    std::string output_json;

    app.add_option("-c, --config", input_json, "input json file");

    CLI11_PARSE(app, argc, argv);
    std::ifstream jsonFile(input_json);
    json config;
    jsonFile >> config;
    // Access the parameters in the JSON file
    std::string input_file = config["input_file"];
    std::string output_folder = config["output_folder"];
    std::string output_file = config["output_file"];
    output_json = config["output_json"];

    FrameMark;

    int image_size = 512;
    image_size = config["image_size"];
    std::string height_map_path = config["height_map_path"];
    std::filesystem::path position_map_path = std::string(config["position_map_path"]);
    // "/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/3channel_normal_position/"
    // "ninja_position.exr";
    std::filesystem::path normal_map_path = std::string(config["normal_map_path"]);
    // "/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/3channel_normal_position/"
    // "ninja_normal.exr";
    std::filesystem::path displaced_image_path = output_folder + "displaced_image.exr";
    double target_l = config["target_edge_length"];

    double target_accuracy = config["target_accuracy"];
    SAMPLING_MODE sampling_mode = SAMPLING_MODE::BICUBIC;
    DISPLACEMENT_MODE displacement_mode = DISPLACEMENT_MODE::MESH_3D;
    sampling_mode = config["sampling_mode"];
    displacement_mode = config["displacement_mode"];
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;
    wrapping_mode = config["wrapping_mode"];
    adaptive_tessellation::ENERGY_TYPE energy_type =
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE;
    adaptive_tessellation::EDGE_LEN_TYPE edge_len_type =
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY;
    energy_type = config["energy_type"];
    edge_len_type = config["edge_len_type"];
    int max_iter = 1;
    max_iter = config["max_iter"];
    bool boundary_parameter_on = true;
    boundary_parameter_on = config["boundary_parameter_on"];

    // Loading the input 2d mesh
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    m.create_paired_seam_mesh_with_offset(input_file, UV, F);

    std::ofstream js_o(output_json);
    auto start_time = lagrange::get_timestamp();

    Image image;
    image.load(height_map_path, wrapping_mode, wrapping_mode);
    wmtk::logger().info("/////height image: {}", height_map_path);


    m.set_output_folder(output_folder);
    m.mesh_parameters.m_position_normal_paths = {position_map_path, normal_map_path};

    // m.mesh_preprocessing(input_file, displaced);

    m.mesh_parameters.m_position_normal_paths = {"/home/yunfan/seamPyramid_position.exr",
                                                 "/home/yunfan/seamPyramid_normal_smooth.exr"};
    assert(m.check_mesh_connectivity_validity());
    // stop after 100 iterations
    m.mesh_parameters.m_early_stopping_number = 100;
    m.set_parameters(
        0.00001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    m.split_all_edges();
    assert(m.check_mesh_connectivity_validity());
    m.mesh_parameters.js_log["input"] = input_file;
    m.mesh_parameters.js_log["output"] = output_file;
    m.mesh_parameters.js_log["num_vert"] = UV.rows();
    m.mesh_parameters.js_log["num_faces"] = F.rows();

    wmtk::logger().info("/////target edge length: {}", target_l);
    wmtk::logger().info("/////target accuracy: {}", target_accuracy);

    wmtk::logger().info("/////sampling mode: {}", sampling_mode);
    wmtk::logger().info("/////dispalcement mode: {}", displacement_mode);


    wmtk::logger().info("/////energy type: {}", energy_type);
    wmtk::logger().info("/////energy length type: {}", edge_len_type);

    m.set_parameters(
        target_accuracy,
        target_l,
        image,
        wrapping_mode,
        sampling_mode,
        displacement_mode,
        energy_type,
        edge_len_type,
        boundary_parameter_on);

    m.set_vertex_world_positions(); // compute 3d positions for each vertex

    m.mesh_improvement(max_iter);
    m.consolidate_mesh();

    auto finish_time = lagrange::get_timestamp();
    auto duration = lagrange::timestamp_diff_in_seconds(start_time, finish_time);
    wmtk::logger().info("!!!!finished {}!!!!", duration);
    m.mesh_parameters.js_log["total_time"] = duration;
    m.write_displaced_obj(output_file, m.mesh_parameters.m_displacement);
    // Save the optimized mesh
    wmtk::logger().info("/////output : {}", output_file);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
    return 0;
}
