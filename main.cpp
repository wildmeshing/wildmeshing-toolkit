#include <CLI/CLI.hpp>
#include <chrono>
#include <filesystem>
#include <wmtk/components/run_components.hpp>
#include <wmtk/utils/Logger.hpp>
#ifdef WMTK_RECORD_OPERATIONS
#include <wmtk/Record_Operations.hpp>
#endif
using json = nlohmann::json;

int main(int argc, char** argv)
{
    using path = std::filesystem::path;

    CLI::App app{argv[0]};

    app.ignore_case();

    path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")->required(true);
    bool is_strict = true;
    // app.add_flag("--ns", is_strict, "Disables strict validation of input JSON");


    CLI11_PARSE(app, argc, argv);
    if (!std::filesystem::exists(json_input_file)) {
        wmtk::logger().critical("File `{}` does not exist.", std::string(json_input_file));
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

#ifdef WMTK_RECORD_OPERATIONS
    OperationLogPath = generatePathNameWithCurrentTime();
#endif

    const auto start = std::chrono::high_resolution_clock::now();

    wmtk::components::run_components(spec_json, is_strict);

    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    wmtk::logger().info("Wildmeshing runtime: {} ms", duration.count());

    return EXIT_SUCCESS;
}
