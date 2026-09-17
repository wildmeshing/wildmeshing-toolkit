#include "prismatic_mesh.hpp"

#include <jse/jse.h>
#include <algorithm>
#include <prismatic_mesh_spec.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

namespace wmtk::components::prismatic_mesh {

void prismatic_mesh(nlohmann::json json_params)
{
    const auto spec = jse::embed::wmtk_prismatic_mesh_spec::prismatic_mesh_spec::spec();
    jse::JSE spec_engine;
    if (!spec_engine.verify_json(json_params, spec)) {
        log_and_throw_error(spec_engine.log2str());
    }

    json_params = spec_engine.inject_defaults(json_params, spec);
    const auto path = utils::resolve_path(
        json_params["input_dir"].get<std::string>(),
        json_params["input"].get<std::string>());
    const auto output_name = json_params["output"].get<std::string>();
    if (output_name.empty()) log_and_throw_error("Output filename must not be empty.");
    auto output_path =
        utils::resolve_path(json_params["input_dir"].get<std::string>(), output_name);
    if (!output_path.has_extension()) output_path += ".vtu";
    if (output_path.extension() != ".vtu") {
        log_and_throw_error("Prismatic mesh output must be a .vtu file.");
    }
    if (std::filesystem::weakly_canonical(path) == std::filesystem::weakly_canonical(output_path)) {
        log_and_throw_error("Input and output must be different files.");
    }
    auto input = load_prismatic_mesh(path);
    logger().info("Loaded prismatic mesh: {}", path.string());
    logger().info(
        "{} vertices, {} tetrahedra; {} input vertices, {} offset vertices with correspondence",
        input.vertices.rows(),
        input.tetrahedra.rows(),
        input.input_vertices.size(),
        input.offset_vertices.size());
    logger().info(
        "Input cells: {}; offset band cells: {}",
        std::count(input.input_cells.begin(), input.input_cells.end(), 1),
        std::count(input.offset_tet_tags.begin(), input.offset_tet_tags.end(), 1));
    prism_main(input);
    write_prismatic_mesh(input, output_path);
    logger().info("Wrote result mesh: {}", output_path.string());
}

} // namespace wmtk::components::prismatic_mesh
