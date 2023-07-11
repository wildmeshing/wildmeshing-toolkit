#include <wmtk_components/mesh_info/mesh_info.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_mesh_info", "[components,mesh_info]")
{
    SECTION("should pass")
    {
        json component_json = {{"type", "mesh_info"}, {"input", data_dir / "bunny.off"}};

        std::map<std::string, std::filesystem::path> files;

        CHECK_NOTHROW(wmtk::components::mesh_info(component_json, files));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "mesh_info"},
            {"input", "In case you ever name your file like that: What is wrong with you?"}};

        std::map<std::string, std::filesystem::path> files;

        CHECK_NOTHROW(wmtk::components::mesh_info(component_json, files));
    }
}