
#include <jse/jse.h>
#include <CLI/App.hpp>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include <wmtk/components/procedural/ProceduralOptions.hpp>
#include <wmtk/components/procedural/make_mesh.hpp>
#include "spec.hpp"


int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    nlohmann::json j;
    {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, delaunay_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, delaunay_spec);
        }
    }

    const fs::path input_path_in = j["input_path"];

    const fs::path root = input_path_in.empty() ? json_input_file : input_path_in;

    fs::path input_file = wmtk::components::utils::resolve_path(j["input"], root);

    auto mesh = wmtk::components::input(input_file);

    wmtk::components::procedural::ProceduralOptions options = j.get<wmtk::components::procedural::ProceduralOptions>();


    std::shared_ptr<Mesh> mesh = std::visit(
        [](auto&& op) { return wmtk::components::procedural::make_mesh(op); },
        options.settings);

    return 0;
}

void procedural(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ProceduralOptions options = j.get<ProceduralOptions>();


    std::shared_ptr<Mesh> mesh = std::visit(
        [](auto&& op) { return wmtk::components::internal::procedural::make_mesh(op); },
        options.settings);
    // mesh = create(options);
    if (!bool(mesh)) {
        throw std::runtime_error("Did not obtain a mesh when generating a procedural one");
    }

    cache.write_mesh(*mesh, options.name);
}
