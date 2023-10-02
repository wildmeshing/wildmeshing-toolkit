#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/output/output.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_output", "[components][output][.]")
{
    std::map<std::string, std::filesystem::path> files;

    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 2},
        {"file", data_dir / "bunny.off"}};

    wmtk::components::input(input_component_json, files);

    SECTION("should pass")
    {
        json component_json = {
            {"type", "output"},
            {"input", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "bunny"}};

        CHECK_NOTHROW(wmtk::components::output(component_json, files));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "output"},
            {"input", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "unknown file ending.abcdef"}};

        CHECK_THROWS(wmtk::components::output(component_json, files));
    }
}
