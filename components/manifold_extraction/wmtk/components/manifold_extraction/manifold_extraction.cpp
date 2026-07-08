#include "manifold_extraction.hpp"
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <manifold_extraction_spec.hpp>
#include <vector>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>
#include "ManExtractTetMesh.h"
#include "ManExtractTriMesh.h"
#include "Parameters.h"

using namespace wmtk::components::simwild;
using namespace wmtk::components;

namespace wmtk::components::manifold_extraction {


void manifold_extraction(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec =
            jse::embed::wmtk_manifold_extraction_spec::manifold_extraction_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    // load input file path
    std::string input_path = resolve_path(root, json_params["input"]).string();

    // load params
    Parameters man_params(json_params);
    const std::string tag_selection_str = json_params["tag_selection"];
    std::filesystem::path output_filename = man_params.output_path;
    int NUM_THREADS = 0;

    // debugging output filename
    std::filesystem::path debug_outfilename = output_filename;
    debug_outfilename.replace_extension("");

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // read image / MSH
    InputData input_data = read_image_msh(input_path);
    man_params.init(
        input_data.V_input.colwise().minCoeff(),
        input_data.V_input.colwise().maxCoeff());
    topological_offset::Parameters offset_params = man_params.generate_offset_params();


    // split into 2d and 3d cases
    if (input_data.T_input.cols() == 3) { // input is 2d trimesh
        logger().info("Input mesh (2d trimesh): {}", input_path);

        // initialize mesh
        ManExtractTriMesh mesh(man_params, offset_params, NUM_THREADS);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names);

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // start timer
        igl::Timer timer;
        timer.start();

        // label non manifold components as offset input
        mesh.m_man_params.tag_selection =
            expression_parser::parse(tag_selection_str, mesh.m_tag_name_to_id);
        size_t num_nmv = mesh.label_non_manifold();
        logger().info("\tNonmanifold vertices: {}", num_nmv);
        if (mesh.m_man_params.debug_output) { // save input complex
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // create offset
        mesh.execute_offset(output_filename);

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // write curve mesh
        if (mesh.m_man_params.write_surface) {
            logger().info("Extracting curve mesh...");
            mesh.write_curve(output_filename.string());
        }

        // report
        const std::string report_file = json_params["report"];
        if (!report_file.empty()) {
            std::ofstream f_out(report_file);
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            f_out << std::setw(4) << report;
            f_out.close();
        }

        // save output
        mesh.write_msh_groups(output_filename.string());
        if (mesh.m_man_params.save_vtu) {
            mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }
        wmtk::logger().info("======= finish =========");
    } else { // input is 3d tetmesh
        logger().info("Input mesh (3d tetmesh): {}", input_path);

        // initialize mesh
        ManExtractTetMesh mesh(man_params, offset_params, NUM_THREADS);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names);

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // start timer
        igl::Timer timer;
        timer.start();

        // label non manifold components as offset input
        mesh.m_man_params.tag_selection =
            expression_parser::parse(tag_selection_str, mesh.m_tag_name_to_id);
        auto [num_nme, num_nmv] = mesh.label_non_manifold();
        logger().info("\tNonmanifold edges: {}", num_nme);
        logger().info("\tNonmanifold vertices: {}", num_nmv);
        if (mesh.m_man_params.debug_output) { // save input complex
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // create offset
        mesh.execute_offset(output_filename);

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // write surface mesh
        if (mesh.m_man_params.write_surface) {
            logger().info("Extracting surface mesh...");
            mesh.write_surface(output_filename.string());
        }

        // report
        const std::string report_file = json_params["report"];
        if (!report_file.empty()) {
            std::ofstream f_out(report_file);
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["before #t"] = mesh.m_init_counts[3];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["after #t"] = mesh.get_tets().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            f_out << std::setw(4) << report;
            f_out.close();
        }

        // save output
        mesh.write_msh_groups(output_filename.string());
        if (mesh.m_man_params.save_vtu) {
            mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }
        wmtk::logger().info("======= finish =========");
    }
}


} // namespace wmtk::components::manifold_extraction