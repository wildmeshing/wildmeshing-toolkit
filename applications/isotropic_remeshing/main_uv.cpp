
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/parse_jse.hpp>
#include <wmtk/components/input/InputOptions.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshingOptions.hpp>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "spec.hpp"
#include "make_multimesh.hpp"

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

    nlohmann::json j = wmtk::applications::utils::parse_jse(
        wmtk::applications::isotropic_remeshing::spec,
        json_input_file);


    const auto input_js =  j["input"];

    const auto input_opts =input_js.get<wmtk::components::input::InputOptions>();


    auto named_mesh = wmtk::components::input::input(input_opts);
    auto mesh_ptr = named_mesh.root().shared_from_this();
    named_mesh.set_mesh(*make_multimesh(*mesh_ptr, input_js));

    wmtk::components::isotropic_remeshing::IsotropicRemeshingOptions options;

    options.load_json(j);

    options.position_attribute =
        mesh_ptr->get_attribute_handle<double>(j["position_attribute"], PrimitiveType::Vertex);
    if(j.contains("inversion_position_attribute")) {
        options.inversion_position_attribute =
            mesh_ptr->get_attribute_handle<double>(j["inversion_position_attribute"].get<std::string>(), PrimitiveType::Vertex);
    }
    for (const auto& other : j["other_position_attributes"]) {
        options.other_position_attributes.emplace_back(
            mesh_ptr->get_attribute_handle<double>(other, PrimitiveType::Vertex));
    }

    wmtk::components::isotropic_remeshing::isotropic_remeshing(options);

    // input uv mesh

    // multimesh the input meshes if not already multimeshed
    // OR - should this be a diff app?

    // call isotropic_remeshing


    const std::string output_path = j["output"];
    wmtk::components::output_hdf5(*mesh_ptr, j["output"]);
}
