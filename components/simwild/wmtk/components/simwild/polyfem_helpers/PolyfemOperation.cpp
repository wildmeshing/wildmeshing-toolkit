#include "PolyfemOperation.hpp"

#include "DeformedMesh.hpp"
#include "MeshReduction.hpp"

#include <jse/jse.h>
#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/DriverPrologue.hpp>

#include <algorithm>

namespace wmtk::components::simwild::polyfem_helpers {

void validate_polyfem_operation(nlohmann::json& json_params)
{
    const nlohmann::json spec = wmtk::components::simwild::simwild_spec_for(json_params);
    jse::JSE spec_engine;
    spec_engine.strict = true;
    if (!spec_engine.verify_json(json_params, spec)) {
        log_and_throw_error(spec_engine.log2str());
    }
    json_params = spec_engine.inject_defaults(json_params, spec);
}

std::string operation_input_path(const nlohmann::json& json_params)
{
    const std::vector<std::string> inputs =
        wmtk::utils::resolve_input_paths(
            json_params,
            json_params["input_dir"].get<std::string>());
    if (inputs.size() != 1) {
        log_and_throw_error(
            "operation {} solves on one already-tagged .msh, but {} input files were given",
            json_params["operation"].get<std::string>(),
            inputs.size());
    }
    return inputs.front();
}

std::filesystem::path sim_dir(const std::string& output, const std::string& subdir)
{
    std::filesystem::path out_dir = std::filesystem::path(output).parent_path();
    if (out_dir.empty()) {
        out_dir = ".";
    }
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path sim = std::filesystem::canonical(out_dir) / subdir;
    std::filesystem::create_directories(sim);
    return std::filesystem::canonical(sim);
}

ReducedMesh reduce_mesh(
    const std::string& input,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir,
    const bool inputs_only)
{
    ReducedMesh out;
    out.path = sim_in_dir / (std::filesystem::path(input).stem().string() + "_polyfem.msh");
    logger().info("[reduce mesh for polyfem]");
    out.content = polyfem_reduced_msh(input, ambient_like_tags);
    if (inputs_only) {
        write_polyfem_reduced_msh(out.path.string(), out.content);
        out.info = get_mesh_info(out.path.string());
    } else {
        out.info = get_mesh_info(out.content);
    }
    logger().info("Reduced material tags : {}  dim={}", out.info.tags, out.info.dim);
    return out;
}

void emit_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    const bool with_collision_proxy,
    const bool inputs_only,
    SolveInputs& memory)
{
    const auto path = [&dir](const char* name) { return (dir / name).string(); };
    logger().info("Writing:");
    // The OBJ is written in every mode, although a normal run hands polyfem the proxy from
    // `memory` and never reads this file: it is the only convenient way to see which faces the
    // selection picked, and the Python engine always wrote it.
    write_collision_mesh_obj(path("interface_collision.obj"), generated.collision_mesh);
    if (inputs_only) {
        write_constraint_hdf5(path("interface_constraint.hdf5"), generated.fitting);
        write_constraint_hdf5(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
        if (with_collision_proxy) {
            write_linear_map_hdf5(path("interface_linear_map.hdf5"), generated.linear_map);
            write_collision_body_ids_txt(
                path("collision_body_ids.txt"),
                generated.collision_body_ids);
        }
        return;
    }
    memory.constraints.emplace(path("interface_constraint.hdf5"), generated.fitting);
    memory.constraints.emplace(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
    if (with_collision_proxy) {
        memory.collision_meshes.emplace(path("interface_collision.obj"), generated.collision_mesh);
        memory.linear_maps.emplace(path("interface_linear_map.hdf5"), generated.linear_map);
        memory.collision_body_ids.emplace(
            path("collision_body_ids.txt"),
            generated.collision_body_ids);
    }
}

bool allow_out_of_iterations(const OrderedJson& doc)
{
    const auto solver = doc.find("solver");
    if (solver == doc.end() || !solver->is_object()) return false;
    const auto nonlinear = solver->find("nonlinear");
    if (nonlinear == solver->end() || !nonlinear->is_object()) return false;
    const auto flag = nonlinear->find("allow_out_of_iterations");
    return flag != nonlinear->end() && flag->get<bool>();
}

std::unique_ptr<PolyfemBackend> operation_backend(PreparedOperation& prepared)
{
    return in_process_backend(std::move(prepared.inputs));
}

void write_operation_result(const PreparedOperation& prepared)
{
    logger().info("[write deformed msh]");
    write_deformed_msh(
        prepared.input,
        prepared.sim_out_dir / "solution.txt",
        prepared.output + ".msh",
        prepared.cfg["scale"].get<double>());
}

} // namespace wmtk::components::simwild::polyfem_helpers
