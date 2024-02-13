#include <CLI/CLI.hpp>
#include <filesystem>
#include <wmtk/components/run_components.hpp>
#include <wmtk/utils/Logger.hpp>

using json = nlohmann::json;

int main(int argc, char** argv)
{
    using path = std::filesystem::path;

    CLI::App app{argv[0]};

    app.ignore_case();

    path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")->required(true);
    bool is_strict = false;
    // app.add_flag("--ns", is_strict, "Disables strict validation of input JSON");


    CLI11_PARSE(app, argc, argv);
    if (!std::filesystem::exists(json_input_file)) {
        wmtk::logger().critical("File `{}` does not exist.", json_input_file);
        return EXIT_FAILURE;
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

    wmtk::components::run_components(spec_json, is_strict);

    return EXIT_SUCCESS;
}
