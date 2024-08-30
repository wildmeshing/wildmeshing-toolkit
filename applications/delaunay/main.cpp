#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>

#include "delaunay_spec.hpp"

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


    auto mesh = wmtk::components::input(j["input"]);

    wmtk::logger().info("mesh has {} vertices", mesh->get_all(PrimitiveType::Vertex).size());

    return 0;
}
