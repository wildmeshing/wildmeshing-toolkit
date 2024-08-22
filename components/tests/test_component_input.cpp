#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/input/internal/mesh_with_tag_from_image.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::base;
using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_input", "[components][input]")
{
    wmtk::io::Cache cache;

    SECTION("should pass")
    {
        const std::filesystem::path input_file = data_dir / "small.msh";
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()}};


        CHECK_NOTHROW(wmtk::components::input(Paths(), component_json, cache));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", "In case you ever name your file like that: What is wrong with you?"},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()}};

        CHECK_THROWS(wmtk::components::input(Paths(), component_json, cache));
    }
}

TEST_CASE("component_input_point", "[components][input][.]")
{
    wmtk::io::Cache cache;

    // TODO we need a point cloud to read from
    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"file", (data_dir / "point_clouds" / "bunny_pts.msh").string()},
        {"ignore_z", false},
        {"tetrahedron_attributes", json::array()}};

    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::input(Paths(), component_json, cache));
}

TEST_CASE("mesh_with_tag_from_image", "[components][input]")
{
    using namespace wmtk;
    io::Cache cache;

    std::filesystem::path img_path = data_dir / "images/half_white_half_black.png";

    const std::string tag_name = "img_tag";

    std::shared_ptr<TriMesh> m;

    REQUIRE_NOTHROW(m = components::internal::mesh_with_tag_from_image(img_path, tag_name));

    if (false) {
        ParaviewWriter writer("mesh_with_tag_from_image", "vertices", *m, true, true, true, false);
        m->serialize(writer);
    }
}