#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>

using namespace wmtk::components::base;

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
        {"ignore_z", false},
        {"tetrahedron_attributes", json::array()}};

    wmtk::components::input(Paths(), input_component_json, cache);

    SECTION("should pass")
    {
        json component_json = R"({
            "input": "input_mesh",
            "attributes": {"position": "vertices"},
            "file": "bunny"
        })"_json;

        CHECK_NOTHROW(wmtk::components::output(Paths(), component_json, cache));
    }

    SECTION("should throw")
    {
        json component_json = R"({
            "input": "input_mesh",
            "attributes": {"position": "vertices"},
            "file": "unknown file ending.abcdef"
        })"_json;

        CHECK_THROWS(wmtk::components::output(Paths(), component_json, cache));
    }
}
