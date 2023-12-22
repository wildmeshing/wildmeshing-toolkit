#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk_components/input/input.hpp>
// TODOfix: reinclude me
// #include <wmtk_components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/output/output.hpp>

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
        exit(-1);
    }

    const path wmtk_spec_file = WMTK_APP_INPUT_SPEC;
    json rules_json;
    {
        std::ifstream f(wmtk_spec_file);
        if (!f.is_open()) {
            spdlog::error("Could not open wmtk specification file: {}", wmtk_spec_file.string());
            return -1;
        }
        rules_json = json::parse(f);
    }

    json spec_json;
    {
        std::ifstream f(json_input_file);
        if (!f.is_open()) {
            wmtk::logger().error("Could not open json file: {}", json_input_file.string());
            return -1;
        }
        spec_json = json::parse(f);
    }

    jse::JSE spec_engine;
    bool r = spec_engine.verify_json(spec_json, rules_json);
    if (!r) {
        wmtk::logger().error("{}", spec_engine.log2str());
        return -1;
    } else {
        spec_json = spec_engine.inject_defaults(spec_json, rules_json);
    }

    {
        std::ofstream o("debug_output.json");
        o << std::setw(4) << spec_json << std::endl;
        o.close();
    }

    std::map<
        std::string,
        std::function<void(const nlohmann::json&, std::map<std::string, std::filesystem::path>&)>>
        components;

    // register components
    components["input"] = wmtk::components::input;
    components["mesh_info"] = wmtk::components::mesh_info;
    // TODOfix: reinclude me
    //  components["isotropic_remeshing"] = wmtk::components::isotropic_remeshing;
    components["output"] = wmtk::components::output;

    std::map<std::string, std::filesystem::path> files;

    // iterate through components array
    for (const json& component_json : spec_json["components"]) {
        spdlog::info("Component {}", component_json["type"]);
        components[component_json["type"]](component_json, files);
    }


    return 0;
}
