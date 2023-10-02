#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_mesh_info", "[components][mesh_info][.]")
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
        json mesh_info_component_json = {{"type", "mesh_info"}, {"input", "input_mesh"}};
        CHECK_NOTHROW(wmtk::components::mesh_info(mesh_info_component_json, files));
    }

    SECTION("should throw")
    {
        json mesh_info_component_json = {
            {"type", "mesh_info"},
            {"input", "In case you ever name your file like that: What is wrong with you?"}};

        CHECK_THROWS(wmtk::components::mesh_info(mesh_info_component_json, files));
    }
}
