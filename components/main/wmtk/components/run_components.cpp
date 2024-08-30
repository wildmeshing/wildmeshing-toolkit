#include "run_components.hpp"

#include <jse/jse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Stopwatch.hpp>


#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include <fstream>

//inside ${CMAKE_CURRENT_BINARY_DIR}/autogen
#include <components_include.hpp>

namespace wmtk::components {


wmtk::io::Cache run_components(const nlohmann::json& json_input_file, bool strict)
{
    const std::filesystem::path wmtk_spec_file = WMTK_APP_INPUT_SPEC;
    nlohmann::json rules_json;
    {
        std::ifstream f(wmtk_spec_file);
        if (!f.is_open()) {
            log_and_throw_error(
                "Could not open wmtk specification file: {}",
                wmtk_spec_file.string());
        }
        rules_json = nlohmann::json::parse(f);
    }

    jse::JSE spec_engine;
    spec_engine.strict = strict;

//inside ${CMAKE_CURRENT_BINARY_DIR}/autogen
#include <spec_include.hpp>
    rules_json = spec_engine.inject_include(rules_json);

    // std::cout << rules_json.dump(2) << std::endl;
    nlohmann::json spec_json;
    bool r = spec_engine.verify_json(json_input_file, rules_json);
    if (!r) {
        log_and_throw_error("{}", spec_engine.log2str());
    } else {
        spec_json = spec_engine.inject_defaults(json_input_file, rules_json);
    }

    // {
    //     std::ofstream o("debug_output.json");
    //     o << std::setw(4) << spec_json << std::endl;
    //     o.close();
    // }


    if (!wmtk::has_user_overloaded_logger_level()) {
        wmtk::logger().set_level(spec_json["settings"]["log_level"]);
        wmtk::opt_logger().set_level(spec_json["settings"]["opt_log_level"]);
    }

    const std::string root_path = spec_json["root_path"];

    const std::string output_dir =
        wmtk::utils::resolve_path(spec_json["output"]["directory"], root_path, false);
    if (!output_dir.empty()) {
        std::filesystem::create_directories(output_dir);
    }

    std::map<
        std::string,
        std::function<void(const utils::Paths&, const nlohmann::json&, wmtk::io::Cache&)>>
        components;

// register components
// inside ${CMAKE_CURRENT_BINARY_DIR}/autogen
#include <components_map.hpp>

    utils::Paths paths;
    paths.root_path = root_path;
    paths.output_dir = output_dir;

    logger().info("Root path: {}, output dir: {}", root_path, output_dir);


    wmtk::io::Cache cache(spec_json["output"]["cache"], output_dir);

    // iterate through components array
    for (const nlohmann::json& component_json : spec_json["components"]) {
        for (auto& el : component_json.items()) {
            wmtk::logger().info("Component {}", el.key());
            wmtk::utils::StopWatch sw(el.key());
            components[el.key()](paths, el.value(), cache);
            cache.flush_multimeshes();
        }
    }

    return cache;
}
} // namespace wmtk::components
