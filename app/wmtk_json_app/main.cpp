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

    std::map<std::string, std::filesystem::path> files;

    // iterate through components array
    for (const json& component_json : spec_json["components"]) {
        for (const auto& [component_name, component_options] : component_json.items()) {
            if (component_name == "input") {
                wmtk::components::input(component_options, files);
            } else if (component_name == "mesh_info") {
                wmtk::components::mesh_info(component_options, files);
            }
        }
    }


    return 0;
}
