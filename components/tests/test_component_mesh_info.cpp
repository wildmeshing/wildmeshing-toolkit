#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/mesh_info/mesh_info.hpp>
#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/io/Cache.hpp>

using namespace wmtk::components::utils;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_mesh_info", "[components][mesh_info]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    const std::filesystem::path input_file = data_dir / "small.msh";

    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"file", input_file.string()},
        {"ignore_z", false},
        {"tetrahedron_attributes", json::array()}};


    wmtk::components::input(Paths(), input_component_json, cache);


    SECTION("should pass")
    {
        json mesh_info_component_json = {{"type", "mesh_info"}, {"input", "input_mesh"}};
        CHECK_NOTHROW(wmtk::components::mesh_info(Paths(), mesh_info_component_json, cache));
    }

    SECTION("should throw")
    {
        json mesh_info_component_json = {
            {"type", "mesh_info"},
            {"input", "In case you ever name your file like that: What is wrong with you?"}};

        CHECK_THROWS(wmtk::components::mesh_info(Paths(), mesh_info_component_json, cache));
    }
}
