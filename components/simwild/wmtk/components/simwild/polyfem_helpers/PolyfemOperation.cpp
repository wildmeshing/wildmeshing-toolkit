#include "PolyfemOperation.hpp"

#include "MeshReduction.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::simwild::polyfem_helpers {

std::string operation_input(const nlohmann::json& params)
{
    const nlohmann::json& inputs = params["input"];
    if (inputs.size() != 1) {
        log_and_throw_error(
            "operation {} solves on one already-tagged .msh, but {} input files were given",
            params["operation"].get<std::string>(),
            inputs.size());
    }
    return inputs.front().get<std::string>();
}

std::filesystem::path generated_dir(const nlohmann::json& params, const std::string& subdir)
{
    std::filesystem::path out_dir =
        std::filesystem::path(params["output"].get<std::string>()).parent_path();
    if (out_dir.empty()) {
        out_dir = ".";
    }
    return out_dir / subdir;
}

ReducedMesh reduce_mesh(
    const std::string& input,
    const TaggedMesh& mesh,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir)
{
    ReducedMesh out;
    out.path = sim_in_dir / (std::filesystem::path(input).stem().string() + "_polyfem.msh");
    logger().info("[reduce mesh for polyfem]");
    out.content = polyfem_reduced_msh(mesh, ambient_like_tags);
    out.info = get_mesh_info(out.content);
    logger().info("Reduced material tags : {}  dim={}", out.info.tags, out.info.dim);
    return out;
}

void add_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    const bool with_collision_proxy,
    SolveInputs& files)
{
    const auto path = [&dir](const char* name) { return (dir / name).string(); };
    files.constraints.emplace(path("interface_constraint.hdf5"), generated.fitting);
    files.constraints.emplace(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
    files.collision_meshes.emplace(path("interface_collision.obj"), generated.collision_mesh);
    if (with_collision_proxy) {
        files.linear_maps.emplace(path("interface_linear_map.hdf5"), generated.linear_map);
        files.collision_body_ids.emplace(
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

SolveReport solve_report(
    const OrderedJson& sim_json,
    const SolveResult& result,
    const std::optional<int64_t> iteration)
{
    SolveReport report;
    report.iteration = iteration;
    report.sim_json = sim_json;
    report.active_distance = result.active_distance;
    report.statuses = result.statuses;
    report.log = result.lines;
    return report;
}

} // namespace wmtk::components::simwild::polyfem_helpers
