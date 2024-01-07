#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/base/resolve_path.hpp>

#include "components_include.hpp"

using namespace wmtk;
using namespace wmtk::components::base;

using json = nlohmann::json;

int main(int argc, char** argv)
{
    using path = std::filesystem::path;

    CLI::App app{argv[0]};
    path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")->required(true);


    CLI11_PARSE(app, argc, argv);
    if (!std::filesystem::exists(json_input_file)) {
        wmtk::logger().critical("File `{}` does not exist.", json_input_file);
        return EXIT_FAILURE;
    }

    const path wmtk_spec_file = WMTK_APP_INPUT_SPEC;
    json rules_json;
    {
        std::ifstream f(wmtk_spec_file);
        if (!f.is_open()) {
            wmtk::logger().error(
                "Could not open wmtk specification file: {}",
                wmtk_spec_file.string());
            return EXIT_FAILURE;
        }
        rules_json = json::parse(f);
    }

    json spec_json;
    {
        std::ifstream f(json_input_file);
        if (!f.is_open()) {
            wmtk::logger().error("Could not open json file: {}", json_input_file.string());
            return EXIT_FAILURE;
        }
        spec_json = json::parse(f);
    }
    if (!spec_json.contains("root_path")) spec_json["root_path"] = json_input_file;


    jse::JSE spec_engine;

#include "spec_include.hpp"
    rules_json = spec_engine.inject_include(rules_json);

    // std::cout << rules_json << std::endl;

    bool r = spec_engine.verify_json(spec_json, rules_json);
    if (!r) {
        wmtk::logger().error("{}", spec_engine.log2str());
        return EXIT_FAILURE;
    } else {
        spec_json = spec_engine.inject_defaults(spec_json, rules_json);
    }

    // {
    //     std::ofstream o("debug_output.json");
    //     o << std::setw(4) << spec_json << std::endl;
    //     o.close();
    // }

    wmtk::logger().set_level(spec_json["settings"]["log_level"]);
    wmtk::opt_logger().set_level(spec_json["settings"]["opt_log_level"]);

    const std::string root_path = spec_json["root_path"];

    const std::string output_dir =
        wmtk::components::base::resolve_path(spec_json["output"]["directory"], root_path, false);
    if (!output_dir.empty()) {
        std::filesystem::create_directories(output_dir);
    }

    std::
        map<std::string, std::function<void(const Paths&, const nlohmann::json&, wmtk::io::Cache&)>>
            components;

// register components
#include "components_map.hpp"

    Paths paths;
    paths.root_path = root_path;
    paths.output_dir = output_dir;

    logger().info("Root path: {}, output dir: {}", root_path, output_dir);


    wmtk::io::Cache cache(spec_json["output"]["cache"], output_dir);

    // iterate through components array
    for (const json& component_json : spec_json["components"]) {
        for (auto& el : component_json.items()) {
            wmtk::logger().info("Component {}", el.key());
            components[el.key()](paths, el.value(), cache);
        }
    }


    return 0;
}
