#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/parse_jse.hpp>
#include <wmtk/components/input/InputOptions.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/isotropic_remeshing/RegulatedIsotropicRemeshingOptions.hpp>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing_regulated.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "spec_regulated.hpp"
#ifdef WMTK_RECORD_OPERATIONS
#include <wmtk/Record_Operations.hpp>
#endif

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
        wmtk::applications::isotropic_remeshing::regulated_spec,
        json_input_file);

#ifdef WMTK_RECORD_OPERATIONS
    std::string model_name = "default";
    if (j["input"]["file"].is_string()) {
        fs::path input_path = j["input"]["file"].get<std::string>();
        if (input_path.has_filename()) {
            model_name = input_path.stem().string();
        }
    }
    std::cout << "Model name: " << model_name << std::endl;
    OperationLogPath = generatePathNameWithModelName(model_name);
    initializeBatchLogging();
#endif

    const auto input_opts = j["input"].get<wmtk::components::input::InputOptions>();

    auto named_mesh = wmtk::components::input::input(input_opts);
    auto mesh_ptr = named_mesh.root().shared_from_this();

    wmtk::components::isotropic_remeshing::RegulatedIsotropicRemeshingOptions options;
    options.load_json(j);

    const auto& js_attrs = j["attributes"];
    options.position_attribute =
        mesh_ptr->get_attribute_handle<double>(js_attrs["position"], PrimitiveType::Vertex);
    if (auto opt = js_attrs["position"]; !opt.empty()) {
        options.inversion_position_attribute =
            mesh_ptr->get_attribute_handle<double>(opt, PrimitiveType::Vertex);
    }
    for (const auto& other : js_attrs["other_positions"]) {
        options.other_position_attributes.push_back(
            mesh_ptr->get_attribute_handle<double>(other, PrimitiveType::Vertex));
    }

    wmtk::components::isotropic_remeshing::isotropic_remeshing_regulated(options);

    const std::string output_path = j["output"];
    wmtk::components::output::output(*mesh_ptr, j["output"], options.position_attribute);

#ifdef WMTK_RECORD_OPERATIONS
    finalizeBatchLogging();
#endif

    return 0;
}
