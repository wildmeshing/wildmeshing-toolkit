#include "topological_offset.hpp"

#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "Parameters.h"
#include "TopoOffsetMesh.h"
#include "read_image_msh.hpp"

#include "file_generation.cpp"  // DEVELOPMENT

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
    randMesh();
    return;

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
    Parameters params;
    params.tag_label = json_params["tag_label"];  // tag used to id components
    for (int tag : json_params["sep_tags"]) {
        params.sep_tags.push_back(tag);
    }
    params.fill_tag = json_params["fill_tag"];
    params.manifold_mode = json_params["manifold_mode"];
    params.manifold_union = json_params["manifold_union"];
    params.output_path = resolve_path(root, json_params["output"]).string();
    std::filesystem::path output_filename = params.output_path;
    int NUM_THREADS = json_params["num_threads"];
    const bool write_vtu = json_params["write_vtu"];
    params.debug_output = json_params["DEBUG_output"];

    // input file must be .msh
    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // read image / MSH
    MatrixXd V_input;
    MatrixXi T_input;
    MatrixXi T_input_tags;
    std::map<std::string, int> tag_label_map;

    // input is a tet mesh
    logger().info("Input mesh: {}", input_path);
    read_image_msh(input_path, V_input, T_input, T_input_tags, tag_label_map);

    // initialize mesh
    topological_offset::TopoOffsetMesh mesh(params, NUM_THREADS);
    mesh.init_from_image(V_input, T_input, T_input_tags, tag_label_map);
    mesh.consolidate_mesh();

    // record counts (mostly debugging, this is probably really slow)
    mesh.init_counts[0] = mesh.vertex_size();
    mesh.init_counts[1] = mesh.get_edges().size();
    mesh.init_counts[2] = mesh.get_faces().size();
    mesh.init_counts[3] = mesh.tet_size();

    igl::Timer timer;
    timer.start();

    // output input complex
    if (mesh.m_params.debug_output) {
        mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }

    // make embedding simplicial (split components per Alg 1)
    bool dummy = mesh.is_simplicially_embedded();
    mesh.simplicial_embedding();
    mesh.consolidate_mesh();
    dummy = mesh.is_simplicially_embedded();
    if (mesh.m_params.debug_output) {  // intermediate output
        mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    }

    // perform offset
    mesh.perform_offset();
    mesh.consolidate_mesh();

    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    // output
    std::ofstream fout(output_filename.string() + ".log");
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

    mesh.write_msh(output_filename.string() + ".msh");
    if (mesh.m_params.debug_output) {  // use numbered vtu format
        mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
    } else if (write_vtu) {
        mesh.write_vtu(output_filename.string());
    }

    wmtk::logger().info("======= finish =========");
}


} // namespace wmtk::components::topological_offset