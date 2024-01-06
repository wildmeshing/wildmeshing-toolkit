#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_output", "[components][output]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    const std::filesystem::path input_file = data_dir / "armadillo.msh";
    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 2},
        {"file", input_file.string()},
        {"ignore_z", false}};

    wmtk::components::input(input_component_json, cache);

    SECTION("should pass")
    {
        json component_json = {
            {"type", "output"},
            {"input", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "bunny"}};

        CHECK_NOTHROW(wmtk::components::output(component_json, cache));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "output"},
            {"input", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "unknown file ending.abcdef"}};

        CHECK_THROWS(wmtk::components::output(component_json, cache));
    }
}
