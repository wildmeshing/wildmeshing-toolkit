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
#include "Parameters.h"
#include "TopoOffsetMesh.h"
#include "read_image_msh.hpp"

#include "file_generation.cpp" // DEVELOPMENT

#include "topological_offset_spec.hpp"

// Enables passing Eigen matrices to fmt/spdlog.
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter
{
};


namespace wmtk::components::topological_offset {


void topological_offset(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, topological_offset_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, topological_offset_spec);
    }

    const std::filesystem::path root = json_params["json_input_file"];

    // load input file path
    std::string input_path = resolve_path(root, json_params["input"]).string();

    // load params
    Parameters params;
    params.tag_name = json_params["tag_name"];
    for (const int& val : json_params["sep_tag_vals"]) {
        params.sep_tag_vals.push_back(val);
    }
    params.offset_tag_val = json_params["offset_tag_val"];
    params.output_path = resolve_path(root, json_params["output"]).string();

    std::filesystem::path output_filename = params.output_path;
    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    // int NUM_THREADS = json_params["num_threads"];
    int NUM_THREADS = 0;
    params.save_vtu = json_params["save_vtu"];
    params.debug_output = json_params["DEBUG_output"];

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // read image / MSH
    MatrixXd V_input;
    MatrixXi T_input;
    MatrixXd T_input_tags;
    // std::map<std::string, int> tag_label_map;
    std::vector<std::string> all_tag_labels;

    // input is a tet mesh
    logger().info("Input mesh: {}", input_path);
    read_image_msh(input_path, V_input, T_input, T_input_tags, params.tag_name, all_tag_labels);

    // initialize mesh
    topological_offset::TopoOffsetMesh mesh(params, NUM_THREADS);
    mesh.init_from_image(V_input, T_input, T_input_tags, all_tag_labels);
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

    // start timer
    igl::Timer timer;
    timer.start();

    // make embedding simplicial (split components per Alg 1)
    logger().info("Creating simplicial embedding...");
    bool dummy = mesh.is_simplicially_embedded(); // internally prints to console
    mesh.simplicial_embedding();
    mesh.consolidate_mesh();
    dummy = mesh.is_simplicially_embedded(); // internally prints to console
    if (mesh.m_params.debug_output) { // intermediate output
        mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }

    // perform offset
    logger().info("Performing offset...");
    mesh.perform_offset();
    mesh.consolidate_mesh();

    // stop timer
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    // output
    std::ofstream fout(output_filename.string() + ".log");
    fout << "before:" << std::endl;
    fout << "\t#t: " << mesh.m_init_counts[3] << std::endl;
    fout << "\t#f: " << mesh.m_init_counts[2] << std::endl;
    fout << "\t#e: " << mesh.m_init_counts[1] << std::endl;
    fout << "\t#v: " << mesh.m_init_counts[0] << std::endl;
    fout << "after:" << std::endl;
    fout << "\t#t: " << mesh.tet_size() << std::endl;
    fout << "\t#f: " << mesh.get_faces().size() << std::endl;
    fout << "\t#e: " << mesh.get_edges().size() << std::endl;
    fout << "\t#v: " << mesh.vertex_size() << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    mesh.write_msh(output_filename.string()); // write .msh
    if (mesh.m_params.debug_output) {
        mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    } else if (mesh.m_params.save_vtu) { // write .vtu
        mesh.write_vtu(output_filename.string());
    }

    wmtk::logger().info("======= finish =========");
}


} // namespace wmtk::components::topological_offset