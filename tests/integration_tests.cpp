#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

// components
#include <components_include.hpp>

using namespace wmtk;
using path = std::filesystem::path;

const std::string tags_integration = "[integration_test]";

const path data_dir = WMTK_DATA_DIR;
const path integration_tests_dir = data_dir / "integration_tests";

nlohmann::json load_json(const path& json_input_file)
{
    // read JSON input file
    nlohmann::json j;
    try {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not load or parse JSON input file \n{}", e.what());
    }

    // add path to input file to the json so that it can be used for relative output paths
    j["json_input_file"] = json_input_file.string();

    return j;
}

void wmtk_wrapper(const nlohmann::json& j)
{
    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include <components_map.hpp>

    // make sure input file contains the application name
    if (!j.contains("application")) {
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // execute
    components_map[app_str](j);
}

void wmtk_wrapper(const path& json_input_file)
{
    const auto j = load_json(json_input_file);
    wmtk_wrapper(j);
}

TEST_CASE("Integration_Tests", tags_integration)
{
    namespace fs = std::filesystem;

    for (const auto& dir_entry : fs::directory_iterator(integration_tests_dir)) {
        const path& f = dir_entry.path();
        if (!f.has_extension() || f.extension() != ".json") {
            continue;
        }
        logger().info(">>>>>>>>>> Integration test: {} <<<<<<<<<<", f.filename().string());
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
}

TEST_CASE("TetWild", tags_integration + "[.]")
{
    const path f = integration_tests_dir / "tetwild.json";
    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(f));
    REQUIRE_NOTHROW(wmtk_wrapper(j));
}