#include <wmtk/components/input/input.h>
#include <wmtk/components/mesh_info/mesh_info.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>


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

    json spec_json;
    {
        std::ifstream f(json_input_file);
        if (!f.is_open()) {
            wmtk::logger().error("Could not open json file: {}", json_input_file.string());
            return -1;
        }
        spec_json = json::parse(f);
    }

    std::map<
        std::string,
        std::function<void(const nlohmann::json&, std::map<std::string, std::filesystem::path>&)>>
        components;

    // register components
    components["input"] = wmtk::components::input;
    components["mesh_info"] = wmtk::components::mesh_info;

    std::map<std::string, std::filesystem::path> files;

    // iterate through components array
    for (const json& component_json : spec_json["components"]) {
        components[component_json["type"]](component_json, files);
    }


    return 0;
}
