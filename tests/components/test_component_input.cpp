#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/input/input.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_input", "[components][input][.]")
{
    SECTION("should pass")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", data_dir / "bunny.off"}};

        std::map<std::string, std::filesystem::path> files;

        CHECK_NOTHROW(wmtk::components::input(component_json, files));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", "In case you ever name your file like that: What is wrong with you?"}};

        std::map<std::string, std::filesystem::path> files;
        CHECK_THROWS(wmtk::components::input(component_json, files));
    }
}