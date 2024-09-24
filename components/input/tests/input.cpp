#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/input/mesh_with_tag_from_image.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using json = nlohmann::json;

namespace {
const std::filesystem::path data_dir = WMTK_DATA_DIR;
}

TEST_CASE("component_input", "[components][input]")
{
    SECTION("should pass")
    {
        const std::filesystem::path input_file = data_dir / "small.msh";


        CHECK_NOTHROW(wmtk::components::input::input(input_file, false, {}));
        auto a = wmtk::components::input::input(input_file, false, {});

        json component_json = {
            {"file", input_file.string()},
            {"old_mode", true},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()}};
        auto opts = component_json.get<wmtk::components::input::InputOptions>();
        CHECK(opts.file == input_file);
        CHECK(opts.ignore_z == false);
        CHECK(opts.old_mode == true);
        CHECK(opts.old_mode == true);
        REQUIRE(opts.imported_attributes.has_value());
        CHECK(opts.imported_attributes.value().size() == 4);
        json js2 = opts;
        for (const auto& v : opts.imported_attributes.value()) {
            CHECK(v.size() == 0);
        }

        CHECK(js2 == component_json);

        auto b = wmtk::components::input::input(opts);

        CHECK(*a == b.root());
    }

    {
        nlohmann::json js = "path";
        REQUIRE(js.is_string());
        auto opts = js.get<wmtk::components::input::InputOptions>();
        CHECK(opts.file.string() == "path");
    }

    SECTION("should throw")
    {
        //         json component_json = {
        //             {"type", "input"},
        //             {"name", "input_mesh"},
        //             {"file", "In case you ever name your file like that: What is wrong with
        //             you?"},
        //             {"ignore_z", false},
        //             {"tetrahedron_attributes", json::array()}};

        CHECK_THROWS(wmtk::components::input::input("no file exists at this path", false));
    }
}

TEST_CASE("component_input_point", "[components][input][.]")
{
    const std::filesystem::path input_path = data_dir / "point_clouds" / "bunny_pts.msh";
    CHECK_NOTHROW(wmtk::components::input::input(input_path));
}

TEST_CASE("mesh_with_tag_from_image", "[components][input]")
{
    using namespace wmtk;
    io::Cache cache("wmtk_cache", std::filesystem::current_path());

    std::filesystem::path img_path = data_dir / "images/half_white_half_black.png";

    const std::string tag_name = "img_tag";

    std::shared_ptr<TriMesh> m;

    REQUIRE_NOTHROW(m = components::input::mesh_with_tag_from_image(img_path, tag_name));

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
