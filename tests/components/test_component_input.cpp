#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/input/internal/mesh_with_tag_from_image.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_input", "[components][input]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    SECTION("should pass")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", data_dir / "bunny.off"}};


        CHECK_NOTHROW(wmtk::components::input(component_json, cache));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "In case you ever name your file like that: What is wrong with you?"}};

        CHECK_THROWS(wmtk::components::input(component_json, cache));
    }
}

TEST_CASE("component_input_point", "[components][input]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 0},
        {"file", data_dir / "bunny_points.obj"}};

    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::input(component_json, cache));
}

TEST_CASE("mesh_with_tag_from_image", "[components][input]")
{
    using namespace wmtk;
    io::Cache cache("wmtk_cache", std::filesystem::current_path());

    std::filesystem::path img_path = data_dir / "images/half_white_half_black.png";

    const std::string tag_name = "img_tag";

    std::shared_ptr<TriMesh> m;

    REQUIRE_NOTHROW(m = components::internal::mesh_with_tag_from_image(img_path, tag_name));

    ParaviewWriter writer(
        cache.get_cache_path() / "mesh_with_tag_from_image",
        "vertices",
        *m,
        true,
        true,
        true,
        false);
    m->serialize(writer);
}