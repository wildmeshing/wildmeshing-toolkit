
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

    const auto input_opts = j["input"].get<wmtk::components::input::InputOptions>();

    auto named_mesh = wmtk::components::input::input(input_opts);
    auto mesh_ptr = named_mesh.root().shared_from_this();

    wmtk::components::isotropic_remeshing::IsotropicRemeshingOptions options;

    options.load_json(j);

    const auto& js_attrs = j["attributes"];
    options.position_attribute =
        mesh_ptr->get_attribute_handle<double>(js_attrs["position"], PrimitiveType::Vertex);
    if (auto opt = js_attrs["position"]; !opt.empty()) {
        options.inversion_position_attribute =
            mesh_ptr->get_attribute_handle<double>(opt, PrimitiveType::Vertex);
    }
    for (const auto& other : js_attrs["other_positions"]) {
        options.inversion_position_attribute =
            mesh_ptr->get_attribute_handle<double>(other, PrimitiveType::Vertex);
    }

    wmtk::components::isotropic_remeshing::isotropic_remeshing(options);

    // input uv mesh

    // multimesh the input meshes if not already multimeshed
    // OR - should this be a diff app?

    // call isotropic_remeshing


    const std::string output_path = j["output"];
    wmtk::components::output_hdf5(*mesh_ptr, j["output"]);
}
