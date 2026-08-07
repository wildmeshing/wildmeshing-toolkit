#include "prismatic_mesh.hpp"

#include <jse/jse.h>
#include <prismatic_mesh_spec.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <fstream>

namespace wmtk::components::prismatic_mesh {

void prismatic_mesh(nlohmann::json json_params)
{
    {
        const auto spec = jse::embed::wmtk_prismatic_mesh_spec::prismatic_mesh_spec::spec();
        jse::JSE engine;
        if (!engine.verify_json(json_params, spec)) {
            log_and_throw_error(engine.log2str());
        }
        json_params = engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];
    const auto input_path = wmtk::utils::resolve_path(root, json_params["input"]);
    auto output_path = wmtk::utils::resolve_path(root, json_params["output"]);
    if (output_path.extension() != ".msh") output_path.replace_extension(".msh");

    auto parameters = parameters_from_json(json_params);
    if (!parameters.report.empty()) {
        parameters.report = wmtk::utils::resolve_path(root, parameters.report);
    }

    const auto input = read_prismatic_msh(input_path);
    const auto result = generate_prismatic_mesh(input, parameters);
    write_hybrid_msh(output_path, result.mesh);

    if (!parameters.report.empty()) {
        const auto parent = parameters.report.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream stream(parameters.report);
        if (!stream) {
            log_and_throw_error("Unable to write report '{}'.", parameters.report.string());
        }
        stream << std::setw(2) << result.report.to_json() << '\n';
    }

    logger().info(
        "Prismatic mesh: {} tets, {} pyramids, {} prisms -> {}",
        result.mesh.tets.size(),
        result.mesh.pyramids.size(),
        result.mesh.prisms.size(),
        output_path.string());
}

} // namespace wmtk::components::prismatic_mesh
