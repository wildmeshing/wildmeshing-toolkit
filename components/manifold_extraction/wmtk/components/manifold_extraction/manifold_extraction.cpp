#include "manifold_extraction.hpp"

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
#include "ManExtractMesh.h"
#include "Parameters.h"
#include "read_image_msh.hpp"

#include "manifold_extraction_spec.hpp"

// // Enables passing Eigen matrices to fmt/spdlog.
// template <typename T>
// struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
//     : ostream_formatter
// {
// };


namespace wmtk::components::manifold_extraction {


void manifold_extraction(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, manifold_extraction_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, manifold_extraction_spec);
    }

    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

    // load input file path
    std::string input_path = resolve_path(root, json_params["input"]).string();

    // load params
    Parameters params;
    for (std::string i : json_params["in_tag"]) {
        params.in_tag.insert(i);
    }
    for (std::string i : json_params["replace_tag"]) {
        params.replace_tag.insert(i);
    }
    params.manifold_union = json_params["manifold_union"];
    params.output_path = resolve_path(root, json_params["output"]).string();
    params.debug_output = json_params["DEBUG_output"];
    params.write_surface = json_params["write_surface"];
    std::filesystem::path output_filename = params.output_path;
    int NUM_THREADS = 0;

    // debugging output filename
    std::filesystem::path debug_outfilename = output_filename;
    debug_outfilename.replace_extension("");

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // read image / MSH
    logger().info("Input mesh: {}", input_path);
    InputData input_data = read_image_msh(input_path);

    // initialize mesh
    manifold_extraction::ManExtractMesh mesh(params, NUM_THREADS);
    mesh.init_from_image(
        input_data.V_input,
        input_data.T_input,
        input_data.T_input_tags,
        input_data.V_envelope,
        input_data.F_envelope,
        input_data.tag_names);
    mesh.consolidate_mesh();

    // record counts (mostly debugging, this is probably really slow)
    mesh.init_counts[0] = mesh.vertex_size();
    mesh.init_counts[1] = mesh.get_edges().size();
    mesh.init_counts[2] = mesh.get_faces().size();
    mesh.init_counts[3] = mesh.tet_size();

    // output input mesh as vtu
    if (mesh.m_params.debug_output) {
        mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }

    // write surface from input
    if (mesh.m_params.write_surface) {
        mesh.write_surface(debug_outfilename.string() + "_preoffset_surf");
    }

    // start timer
    igl::Timer timer;
    timer.start();

    // identify non manifold components
    logger().info("Identifying nonmanifold components...");
    auto [nm_e_count, nm_v_count] = mesh.label_non_manifold();
    logger().info("\tNonmanifold edges: {}", nm_e_count);
    logger().info("\tNonmanifold vertices: {}", nm_v_count);
    if (mesh.m_params.debug_output) {
        mesh.write_input_complex(debug_outfilename.string() + "_input_complex");
    }

    // make embedding simplicial (split components per Alg 1)
    logger().info("Creating simplicial embedding...");
    bool dummy = mesh.is_simplicially_embedded();
    mesh.simplicial_embedding();
    mesh.consolidate_mesh();
    dummy = mesh.is_simplicially_embedded();
    if (mesh.m_params.debug_output) { // intermediate output
        mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }

    // perform offset
    logger().info("Performing offset...");
    mesh.perform_offset();
    mesh.label_surface_simplices(true);
    mesh.consolidate_mesh();
    mesh.set_offset_tags();

    // get surface mesh
    logger().info("Extracting surface mesh...");
    MatrixXd V_out;
    MatrixXi F_out;
    mesh.extract_surface_mesh(V_out, F_out);
    MatrixXd V_out_reduced;
    MatrixXi F_out_reduced;
    MatrixXi I; // index map, don't actually need
    MatrixXi B; // dummy variable
    igl::remove_unreferenced(V_out, F_out, V_out_reduced, F_out_reduced, I);
    if (!(igl::is_edge_manifold(F_out_reduced) && igl::is_vertex_manifold(F_out_reduced, B))) {
        log_and_throw_error("Extracted surface is not manifold.");
    } else {
        logger().info("Extracted surface manifold check: PASSED");
    }
    if (mesh.m_params.write_surface) {
        // write output surface
        logger().info("Write {}", output_filename.string());
        igl::write_triangle_mesh(output_filename.string() + ".obj", V_out_reduced, F_out_reduced);
    }

    // stop timer
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    // output
    std::ofstream fout(debug_outfilename.string() + ".log");
    fout << "before:" << std::endl;
    fout << "\t#t: " << mesh.init_counts[3] << std::endl;
    fout << "\t#f: " << mesh.init_counts[2] << std::endl;
    fout << "\t#e: " << mesh.init_counts[1] << std::endl;
    fout << "\t#v: " << mesh.init_counts[0] << std::endl;
    fout << "after:" << std::endl;
    fout << "\t#t: " << mesh.tet_size() << std::endl;
    fout << "\t#f: " << mesh.get_faces().size() << std::endl;
    fout << "\t#e: " << mesh.get_edges().size() << std::endl;
    fout << "\t#v: " << mesh.vertex_size() << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    // write vtu
    if (mesh.m_params.debug_output) {
        mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }
    mesh.write_msh_groups(output_filename.string());
    wmtk::logger().info("======= finish =========");
}


} // namespace wmtk::components::manifold_extraction