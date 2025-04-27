
#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/applications/utils/element_count_report.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>

#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "CLI/CLI.hpp"
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/utils/PathResolver.hpp>

#include <wmtk/components/rgb/RGBOptions.hpp>
#include <wmtk/components/rgb/RGB.hpp>

using namespace wmtk::components;
using namespace wmtk::applications;
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

    std::ifstream ifs(json_input_file);

    nlohmann::json j;
    ifs >> j;


    const auto input_opts = j["input"].get<wmtk::components::input::InputOptions>();

    wmtk::components::multimesh::MeshCollection meshes;
    components::utils::PathResolver path_resolver;

    meshes.add_mesh(wmtk::components::input::input(input_opts, path_resolver));
    auto named_mesh = wmtk::components::input::input(input_opts);
    auto mesh_ptr = named_mesh.root().shared_from_this();

    wmtk::components::rgb::RGBOptions options;

    options.load_json(j);

    const auto& js_attrs = j["attributes"];

    wmtk::components::rgb::RGB(meshes, options);


    const std::string output= j["output"];
    wmtk::components::output::output(meshes,output);
}
