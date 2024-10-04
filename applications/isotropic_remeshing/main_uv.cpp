
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/parse_jse.hpp>
#include <wmtk/components/input/MeshCollection.hpp>
#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/utils/get_attribute.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshingOptions.hpp>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "make_multimesh.hpp"
#include "spec.hpp"

using namespace wmtk::components;
using namespace wmtk;
namespace fs = std::filesystem;

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    //nlohmann::json j = wmtk::applications::utils::parse_jse(
    //    wmtk::applications::isotropic_remeshing::spec,
    //    json_input_file);

    std::ifstream ifs(json_input_file);
    nlohmann::json j = nlohmann::json::parse(ifs);

    const auto input_js = j["input"];

    const auto input_opts = input_js.get<wmtk::components::input::InputOptions>();


    wmtk::components::input::MeshCollection mc;
    auto& named_mesh = mc.add_mesh(input_opts);
    auto mesh_ptr = named_mesh.root().shared_from_this();
    spdlog::warn("Input js: {}", input_js.dump(2));
    if(input_js.contains("multimesh")) {
        mesh_ptr = make_multimesh(*mesh_ptr, input_js["multimesh"]);

        spdlog::info("{} children", mesh_ptr->get_all_child_meshes().size());
    }


    wmtk::components::isotropic_remeshing::IsotropicRemeshingOptions options;

    options.load_json(j);

    options.position_attribute = wmtk::components::input::utils::get_attribute(
            mc, j["position_attribute"]);

    if (j.contains("inversion_position_attribute")) {
    options.inversion_position_attribute = wmtk::components::input::utils::get_attribute(
            mc, j["inversion_position_attribute"]);
    }
    if(j.contains("other_position_attributes")) {
    for (const auto& other : j["other_position_attributes"]) {
        options.other_position_attributes.emplace_back(
                wmtk::components::input::utils::get_attribute(
            mc, other));
    }
    }
    if(j.contains("pass_through_attributes")) {
    for (const auto& other : j["pass_through_attributes"]) {
        options.pass_through_attributes.emplace_back(
                wmtk::components::input::utils::get_attribute(
            mc, other));
    }
    }
    for(const auto& attr: options.pass_through_attributes) {
        spdlog::info("Pass through: {}", attr.name());
    }

    wmtk::components::isotropic_remeshing::isotropic_remeshing(options);

    // input uv mesh

    // multimesh the input meshes if not already multimeshed
    // OR - should this be a diff app?

    // call isotropic_remeshing


    const std::string output_path = j["output"];
    wmtk::components::output::output(*mesh_ptr, j["output"]);
}
