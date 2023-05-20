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
#include <wmtk/utils/json_sink.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <regex>
#include <tracy/Tracy.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "AdaptiveTessellation.h"
#include "LoggerDataCollector.h"
#include "Parameters.h"

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;
using json = nlohmann::json;

inline void ensure_path_exists(const std::filesystem::path& f)
{
    if (!std::filesystem::exists(f)) {
        wmtk::logger().critical("File `{}` does not exist.", f);
        exit(-1);
    }
};

// Returns f if it exists or dir/f if that exists. If both do not exist exit.
inline std::filesystem::path get_existing_path(
    const std::filesystem::path& dir,
    const std::filesystem::path& f)
{
    if (std::filesystem::exists(f)) {
        return f;
    }
    if (std::filesystem::exists(dir / f)) {
        return dir / f;
    }
    wmtk::logger().critical("File `{}` does not exist.", f);
    exit(-1);
}

using namespace wmtk;
using namespace adaptive_tessellation;
using namespace lagrange;
int main(int argc, char** argv)
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    using path = std::filesystem::path;

    ZoneScopedN("adaptive_tessellation_main");
    lagrange::enable_fpe();
    CLI::App app{argv[0]};
    path config_json;
    path output_folder;
    bool log_to_stdout = false;

    app.add_option("-c, --config", config_json, "input json file")->required(false);
    app.add_option("-o, --output", output_folder, "output folder")->required(false);
    app.add_flag("--log_to_stdout", log_to_stdout, "write log output also to std out");


    CLI11_PARSE(app, argc, argv);
    if (config_json.empty()) {
        config_json = "config.json";
        wmtk::logger().info("No config file specified. Using default: {}", config_json.string());
    }
    ensure_path_exists(config_json);

    const path input_folder = config_json.parent_path();
    ensure_path_exists(input_folder);

    json config;
    {
        std::ifstream jsonFile(config_json);
        jsonFile >> config;
    }
    // Access the parameters in the JSON file
    const path input_file = get_existing_path(input_folder, config["input_file"]);
    if (output_folder.empty()) {
        output_folder = "./output";
        wmtk::logger().info("No input path specified. Using './output'.");
    }
    std::filesystem::create_directories(output_folder);
    ensure_path_exists(output_folder);
    const path output_file = output_folder / config["output_file"];
    const path output_json = output_folder / config["output_json"];

    FrameMark;

    const int image_size = config["image_size"];
    const path height_map_path = get_existing_path(input_folder, config["height_map"]);
    const path position_map_path = get_existing_path(input_folder, config["position_map"]);
    const path normal_map_path = get_existing_path(input_folder, config["normal_map"]);
    const double target_l = config["target_edge_length"];
    const double target_accuracy = config["target_accuracy"];
    const SAMPLING_MODE sampling_mode = config["sampling_mode"];
    const DISPLACEMENT_MODE displacement_mode = config["displacement_mode"];
    const WrappingMode wrapping_mode = config["wrapping_mode"];
    const adaptive_tessellation::ENERGY_TYPE energy_type = config["energy_type"];
    const adaptive_tessellation::EDGE_LEN_TYPE edge_len_type = config["edge_len_type"];
    const int max_iter = config["max_iter"];
    const bool boundary_parameter_on = config["boundary_parameter_on"];

    wmtk::logger().info("///// height map: {}", height_map_path);
    wmtk::logger().info("///// normal map: {}", normal_map_path);
    wmtk::logger().info("///// position map: {}", position_map_path);

    // Loading the input 2d mesh
    AdaptiveTessellation m;
    // Eigen::MatrixXd UV;
    // Eigen::MatrixXi F;
    // m.create_paired_seam_mesh_with_offset(input_file, UV, F);

    std::ofstream js_o(output_json);
    auto start_time = lagrange::get_timestamp();

    Image height_map;
    height_map.load(height_map_path, wrapping_mode, wrapping_mode);


    m.set_output_folder(output_folder);
    m.mesh_parameters.m_position_normal_paths = {position_map_path, normal_map_path};

    m.mesh_preprocessing(input_file, position_map_path, normal_map_path, height_map_path);

    assert(m.check_mesh_connectivity_validity());

    m.mesh_parameters.js_log["input"] = input_file;
    m.mesh_parameters.js_log["output"] = output_file;
    // m.mesh_parameters.js_log["num_vert"] = UV.rows();
    // m.mesh_parameters.js_log["num_faces"] = F.rows();

    wmtk::logger().info("/////target edge length: {}", target_l);
    wmtk::logger().info("/////target accuracy: {}", target_accuracy);

    wmtk::logger().info("///// target edge length: {}", target_l);
    wmtk::logger().info("///// target accuracy: {}", target_accuracy);

    wmtk::logger().info("///// sampling mode: {}", sampling_mode);
    wmtk::logger().info("///// dispalcement mode: {}", displacement_mode);

    wmtk::logger().info("///// energy type: {}", energy_type);
    wmtk::logger().info("///// energy length type: {}", edge_len_type);

    m.mesh_parameters.ATlogger = wmtk::make_json_file_logger(
        "ATlogger",
        output_folder / "adaptive_tessellation_log.json",
        true,
        log_to_stdout);


    m.set_parameters(
        target_accuracy,
        target_l,
        height_map,
        wrapping_mode,
        sampling_mode,
        displacement_mode,
        energy_type,
        edge_len_type,
        boundary_parameter_on);
    //// TODO DEBUG
    // m.mesh_parameters.m_early_stopping_number = 100;
    ////
    m.set_vertex_world_positions(); // compute 3d positions for each vertex

    {
        LoggerDataCollector ldc;
        ldc.evaluate_mesh(m);
        ldc.log_json_verbose(m, "before_remeshing");
    }

    LoggerDataCollector ldc;
    ldc.start_timer();
    m.split_all_edges();
    ldc.stop_timer();

    m.consolidate_mesh();

    ldc.evaluate_mesh(m);
    ldc.log_json_verbose(m, "after_remeshing");

    auto finish_time = lagrange::get_timestamp();
    auto duration = lagrange::timestamp_diff_in_seconds(start_time, finish_time);
    wmtk::logger().info("!!!!finished {}!!!!", duration);
    m.mesh_parameters.js_log["total_time"] = duration;

    m.write_obj_displaced(output_file);
    // Save the optimized mesh
    wmtk::logger().info("///// output : {}", output_file);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
    return 0;
}
