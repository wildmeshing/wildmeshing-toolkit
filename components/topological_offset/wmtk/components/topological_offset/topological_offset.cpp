#include "topological_offset.hpp"

#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include "Parameters.h"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"

#include <topological_offset_spec.hpp>

using namespace wmtk::components::simwild;


namespace wmtk::components::topological_offset {
void topological_offset(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
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
    Parameters params(json_params);
    const std::string offset_selection_str = json_params["offset_selection"];
    bool check_manifoldness = json_params["check_manifoldness"];

    std::filesystem::path output_filename = params.output_path;
    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    int NUM_THREADS = 0;

    if (params.debug_output) {
        logger().info("====== input parameters =======");
        logger().info("target_distance: {}", params.target_distance);
        logger().info("relative_ball_threshold: {}", params.relative_ball_threshold);
        logger().info("edge_search_term_len: {}", params.edge_search_term_len);
        logger().info("===============================");
    }

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    InputData input_data = read_image_msh(input_path);
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());
    if (input_data.T_input.cols() == 3) { // input is a 2d tri mesh
        logger().info("Input mesh (2D trimesh): {}", input_path);

        // initialize mesh
        TopoOffsetTriMesh mesh(params, NUM_THREADS);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names);

        // label input complex
        mesh.m_params.offset_selection =
            expression_parser::parse(offset_selection_str, mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            mesh.write_msh_groups(output_filename.string());
            return;
        }

        // check for inversions in input mesh
        auto tris_before = mesh.get_faces();
        if (!mesh.invariants(tris_before)) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris_before) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (std::to_string(t.fid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tri ids: {}", bad_tris_str);
        }
        tris_before.clear();

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // set initial counts
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // output input complex and entire mesh as vtu
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.execute_offset(output_filename);
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check
        auto tris = mesh.get_faces();
        bool noninverted = mesh.invariants(tris);
        if (!noninverted) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (" " + std::to_string(t.fid(mesh)));
                }
            }
            // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
            log_and_throw_error("INVERSION DURING OFFSET! bad tri ids: {}", bad_tris_str);
        }

        // offset region manifoldness check
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed.");
            } else {
                // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
                log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
            }
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

        mesh.write_msh_groups(output_filename.string()); // write .msh with physical groups
        if (mesh.m_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    } else { // input is a 3d tet mesh
        logger().info("Input mesh (3D tetmesh): {}", input_path);

        // initialize mesh
        TopoOffsetTetMesh mesh(params, NUM_THREADS);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names);

        // label input complex
        mesh.m_params.offset_selection =
            expression_parser::parse(offset_selection_str, mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            mesh.write_msh_groups(output_filename.string());
            return;
        }

        // check for inversions in input mesh
        auto tets_before = mesh.get_tets();
        if (!mesh.invariants(tets_before)) {
            std::string bad_tets_str = "";
            for (const TetMesh::Tuple& t : tets_before) {
                std::vector<TetMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tets_str += (std::to_string(t.tid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tet ids: {}", bad_tets_str);
        }
        tets_before.clear();

        // initial number of connected components
        size_t initial_num_comps = mesh.flood_fill();
        mesh.reset_connected_components();

        // initialize BVH
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // output input complex and entire mesh as vtu
        if (mesh.m_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.execute_offset(output_filename);
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check
        auto tets = mesh.get_tets();
        bool noninverted = mesh.invariants(tets);
        if (!noninverted) {
            std::string bad_tets_str = "";
            for (const TetMesh::Tuple& t : tets) {
                std::vector<TetMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tets_str += (" " + std::to_string(t.tid(mesh)));
                }
            }
            // mesh.write_msh_groups(output_filename.string()); // DEBUG write .msh anyway
            log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
        }

        // offset region manifoldness check
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed.");
            } else {
                // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
                log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
            }
        }

        // connected components check
        size_t final_num_comps = mesh.flood_fill();
        mesh.reset_connected_components();
        if (final_num_comps != initial_num_comps) {
            log_and_throw_error(
                "# CONNECTED COMPONENTS MISMATCH: {} before, {} after",
                initial_num_comps,
                final_num_comps);
        } else {
            logger().info(
                "connected components check passed. (# components={})",
                initial_num_comps);
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
            report["after #t"] = mesh.tet_size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            f_out << std::setw(4) << report;
            f_out.close();
        }

        // mesh.write_msh(output_filename.string()); // write .msh
        mesh.write_msh_groups(output_filename.string()); // write .msh with physical groups
        if (mesh.m_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    }
}


} // namespace wmtk::components::topological_offset
