#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/HeapTrimmer.hpp>
#include <wmtk/utils/Logger.hpp>

#include "component_specs.hpp"

// components
#include "components_include.hpp"
#include "components_spec_include.hpp"

using namespace wmtk;

namespace {

int run(int argc, char** argv)
{
    CLI::App app{argv[0]};
    std::filesystem::path json_input_file;
    bool list_components = false;
    std::string options_of;

    app.add_option(
           "-j",
           json_input_file,
           "JSON input file. See individual components for further instructions.")
        ->check(CLI::ExistingFile);
    app.add_flag("--list", list_components, "List the components and what each does.");
    app.add_option("--options", options_of, "List the options of a component.");

    CLI11_PARSE(app, argc, argv);

    component_specs::SpecMap components_spec_map;
    // include auto-generated map
#include "components_spec_map.hpp"

    if (list_components) {
        for (const auto& [name, spec] : components_spec_map) {
            std::cout << name << "\n    "
                      << component_specs::first_sentence(
                             component_specs::root_rule(spec()).value("doc", ""))
                      << "\n";
        }
        return 0;
    }
    if (!options_of.empty()) {
        if (components_spec_map.count(options_of) == 0) {
            std::cerr << "Unknown component " << options_of << " (see --list)\n";
            return 1;
        }
        std::cout << options_of << "\n\n"
                  << component_specs::describe(options_of, components_spec_map.at(options_of)());
        return 0;
    }
    if (json_input_file.empty()) {
        std::cerr << app.help();
        return 1;
    }

    // Return freed heap to the system while the component runs; see HeapTrimmer.
    const HeapTrimmer heap_trimmer;

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

    // The JSON Schema an editor validates the file with (see the documentation's JSON Schemas
    // page) is not an option.
    j.erase("$schema");

    // add path to input file to the json so that it can be used for relative output paths
    j["input_dir"] = json_input_file.parent_path().string();

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // Most components ignore an option their spec does not have, so a misspelled one would
    // silently keep its default.
    if (components_spec_map.count(app_str)) {
        for (const auto& key :
             component_specs::unknown_options(components_spec_map.at(app_str)(), j)) {
            logger().warn(
                "{} has no option `{}` (wmtk_app --options {} lists them)",
                app_str,
                key,
                app_str);
        }
    }

    // execute
    components_map[app_str](j);
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    // An error ends the run with a message and exit code 1, not with std::terminate (abort).
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        logger().error("wmtk_app stopped: {}", e.what());
        return 1;
    }
}
