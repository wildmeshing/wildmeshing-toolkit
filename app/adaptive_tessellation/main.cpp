#include <AdaptiveTessellation.h>
#include <igl/Timer.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <lagrange/utils/fpe.h>
#include <lagrange/utils/timing.h>
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

    int image_size = 512;
    image_size = config["image_size"];
    std::string image_path = config["image_path"];
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;
    wrapping_mode = config["wrapping_mode"];
    Image image(image_size, image_size);
    image.load(image_path, wrapping_mode, wrapping_mode);

    // Loading the input mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(input_file, V, F);
    assert(ok);
    wmtk::logger().info("/////input: {}", input_file);
    wmtk::logger().info("/////interpolation wrapping mode: {}", wrapping_mode);
    std::ofstream js_o(output_json);

    auto start_time = lagrange::get_timestamp();
    adaptive_tessellation::AdaptiveTessellation adaptive_tessellation;

    adaptive_tessellation.mesh_parameters.js_log["input"] = input_file;
    adaptive_tessellation.mesh_parameters.js_log["output"] = output_file;

    adaptive_tessellation.create_mesh(V, F);
    adaptive_tessellation.set_output_folder(output_folder);
    assert(adaptive_tessellation.check_mesh_connectivity_validity());
    adaptive_tessellation.mesh_parameters.js_log["num_vert"] = V.rows();
    adaptive_tessellation.mesh_parameters.js_log["num_faces"] = F.rows();

    double target_l = config["target_edge_length"];
    wmtk::logger().info("/////target edge length: {}", target_l);

    adaptive_tessellation::ENERGY_TYPE energy_type =
        adaptive_tessellation::ENERGY_TYPE::EDGE_LENGTH;
    adaptive_tessellation::EDGE_LEN_TYPE edge_len_type =
        adaptive_tessellation::EDGE_LEN_TYPE::PT_PER_PIXEL;
    energy_type = config["energy_type"];
    edge_len_type = config["edge_len_type"];
    wmtk::logger().info("/////energy type: {}", energy_type);
    wmtk::logger().info("/////energy length type: {}", edge_len_type);

    bool boundary_parameter_on = true;
    boundary_parameter_on = config["boundary_parameter_on"];
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    auto displacement_image_double = [&image](const double& u, const double& v) -> double {
        return (image.get(u, v));
    };

    adaptive_tessellation.set_parameters(
        target_l,
        image,
        wrapping_mode,
        edge_len_type,
        energy_type,
        boundary_parameter_on);
    int max_iter = 1;
    max_iter = config["max_iter"];
    adaptive_tessellation.mesh_improvement(max_iter);
    adaptive_tessellation.consolidate_mesh();

    auto finish_time = lagrange::get_timestamp();
    auto duration = lagrange::timestamp_diff_in_seconds(start_time, finish_time);
    wmtk::logger().info("!!!!finished {}!!!!", duration);
    adaptive_tessellation.mesh_parameters.js_log["total_time"] = duration;
    adaptive_tessellation.write_displaced_obj(output_file, displacement_image_double);
    // Save the optimized mesh
    wmtk::logger().info("/////output : {}", output_file);
    js_o << std::setw(4) << adaptive_tessellation.mesh_parameters.js_log << std::endl;
    js_o.close();
    return 0;
}
