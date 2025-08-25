#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/orient.hpp>
using namespace wmtk;

// igl
#include <igl/boundary_loop.h>
#include <igl/readOBJ.h>
#ifdef USE_IGL_VIEWER
#include <igl/stb/read_image.h>
#endif

// applications
#include "track_line_app.hpp"
#include "track_point_app.hpp"
#include "vtu_utils.hpp"


int main(int argc, char** argv)
{
    CLI::App app{"bijective_map_app"};
    path initial_mesh_file;
    path operation_logs_dir;
    path output_mesh_file;
    std::string application_name = "back";
    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    app.add_option("-o, --output", output_mesh_file, "Output mesh file")->required(true);
    app.add_option("-l, --logs", operation_logs_dir, "Operation logs directory")->required(true);
    app.add_option("-a, --app", application_name, "Application name");

    // options for texture transfer application
    path input_obj_file;
    path input_texture_file;
    app.add_option("--input_obj", input_obj_file, "Input obj file");
    app.add_option("--input_texture", input_texture_file, "Input texture file");
    int height_out = 200;
    int width_out = 200;
    app.add_option("--height_out", height_out, "Height of the output image");
    app.add_option("--width_out", width_out, "Width of the output image");

    // options for iso_lines application
    int N = 5;
    bool no_parallel = false;
    bool separate_curve_vtu = false;
    app.add_option("--N", N, "Number of isolines to generate (default: 5)");
    app.add_flag(
        "--no_parallel",
        no_parallel,
        "Disable parallel processing for curve tracking (default: enabled)");
    app.add_flag(
        "--separate_curve_vtu",
        separate_curve_vtu,
        "Generate separate VTU files for each curve (default: combined)");

    CLI11_PARSE(app, argc, argv);

    if (!std::filesystem::exists(initial_mesh_file)) {
        std::cerr << "File `" << initial_mesh_file << "` does not exist." << std::endl;
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(output_mesh_file)) {
        std::cerr << "File `" << output_mesh_file << "` does not exist." << std::endl;
        return EXIT_FAILURE;
    }

    auto init_mesh_ptr = wmtk::read_mesh(initial_mesh_file);
    auto [F_in, V_in] = static_cast<TriMesh&>(*init_mesh_ptr).get_FV();
    std::cout << "F_in size " << F_in.rows() << ", " << F_in.cols() << std::endl;
    std::cout << "V_in size " << V_in.rows() << ", " << V_in.cols() << std::endl;


    Eigen::MatrixXd V_out, Vt_out, Vn_out;
    Eigen::MatrixXi F_out, Ft_out, Fn_out;

    if (output_mesh_file.extension() == ".vtu") {
        std::cout << "\nloading output vtu file..." << std::endl;
        vtu_utils::read_triangle_mesh_from_vtu(output_mesh_file.string(), V_out, F_out);
    } else {
        std::cout << "\nloading output obj file..." << std::endl;
        igl::readOBJ(output_mesh_file.string(), V_out, Vt_out, Vn_out, F_out, Ft_out, Fn_out);
    }
    std::cout << "F_out size" << F_out.rows() << ", " << F_out.cols() << std::endl;
    std::cout << "V_out size" << V_out.rows() << ", " << V_in.cols() << std::endl;


    std::string model_name = "model_name";
    model_name = initial_mesh_file.stem().string();
    std::cout << "model_name: " << model_name << std::endl;

    if (application_name == "texture") {
        if (output_mesh_file.extension() == ".vtu") {
            std::cout << "\noutput mesh is a vtu file, not supported for texture transfer"
                      << std::endl;
            return EXIT_FAILURE;
        }

        Eigen::MatrixXd V_in_obj, Vt_in_obj, Vn_in_obj;
        Eigen::MatrixXi F_in_obj, Ft_in_obj, Fn_in_obj;

        std::cout << "\nloading input obj file..." << std::endl;
        igl::readOBJ(
            input_obj_file.string(),
            V_in_obj,
            Vt_in_obj,
            Vn_in_obj,
            F_in_obj,
            Ft_in_obj,
            Fn_in_obj);

        std::cout << "F_in_obj size " << F_in_obj.rows() << ", " << F_in_obj.cols() << std::endl;
        std::cout << "V_in_obj size " << V_in_obj.rows() << ", " << V_in_obj.cols() << std::endl;

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
        std::cout << "\nloading texture file..." << std::endl;
#ifdef USE_IGL_VIEWER
        igl::stb::read_image(input_texture_file.string(), R, G, B, A);
#endif
        transfer_texture_app(
            R,
            G,
            B,
            A,
            F_in_obj,
            Vt_in_obj,
            Ft_in_obj,
            Vt_out,
            Ft_out,
            F_out,
            operation_logs_dir,
            width_out,
            height_out);
    } else if (application_name == "forward") {
        forward_track_point_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back") {
        back_track_point_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "render") {
        render_index_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back_lines") {
        back_track_one_curve_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back_r") {
        back_track_point_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    } else if (application_name == "iso_lines") {
        Eigen::MatrixXd V_in_obj, Vt_in_obj, Vn_in_obj;
        Eigen::MatrixXi F_in_obj, Ft_in_obj, Fn_in_obj;
        std::cout << "\nloading input obj file..." << std::endl;
        igl::readOBJ(
            input_obj_file.string(),
            V_in_obj,
            Vt_in_obj,
            Vn_in_obj,
            F_in_obj,
            Ft_in_obj,
            Fn_in_obj);
        bool do_parallel = !no_parallel; // Convert no_parallel to do_parallel
        if (do_parallel) {
            std::cout << "parallel mode" << std::endl;
        } else {
            std::cout << "sequential mode" << std::endl;
        }
        forward_track_iso_lines_app(
            V_in,
            F_in,
            Vt_in_obj,
            Ft_in_obj,
            V_out,
            F_out,
            operation_logs_dir,
            N,
            do_parallel,
            model_name);
    } else if (application_name == "check_iso_lines") {
        // check_iso_lines_step_by_step(V_in, F_in, V_out, F_out, operation_logs_dir);
        std::vector<query_curve> curves_in = load_query_curves(model_name + "_curves.in");
        std::vector<query_curve> curves_out = load_query_curves(model_name + "_curves.out");
        check_iso_lines(V_in, F_in, V_out, F_out, curves_in, curves_out);
    } else if (application_name == "plane_curves") {
        forward_track_plane_curves_app(
            V_in,
            F_in,
            V_out,
            F_out,
            operation_logs_dir,
            N,
            !no_parallel,
            model_name,
            separate_curve_vtu);
    }
    return 0;
}