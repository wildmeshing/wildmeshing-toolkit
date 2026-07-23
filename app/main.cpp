#include <CLI/CLI.hpp>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

// components
#include "components_include.hpp"

using namespace wmtk;

int main(int argc, char** argv)
{
    CLI::App app{argv[0]};
    std::filesystem::path json_input_file;

    app.add_option(
           "-j",
           json_input_file,
           "JSON input file. See individual components for further instructions.")
        ->required()
        ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);


    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include "components_map.hpp"

    // read JSON input file
    nlohmann::json j;
    try {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);
    } catch (const std::exception& e) {
        logger().error("Could not load or parse JSON input file");
        logger().error(e.what());
    }

    // make sure input file contains the application name
    if (!j.contains("application")) {
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    // add path to input file to the json so that it can be used for relative output paths
    j["input_dir"] = json_input_file.parent_path().string();

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // execute
    try {
        components_map[app_str](j);
    } catch (const std::exception& e) {
        // Without this, any error a component throws -- an empty or unreadable input mesh, a
        // failed invariant, and so on -- escapes main, so the runtime calls std::terminate()
        // and the process aborts. Catch it and exit nonzero with a readable message instead
        // of crashing (and, on macOS, spraying a crash report per failure).
        logger().error("Application '{}' failed: {}", app_str, e.what());
        return 1;
    }
}