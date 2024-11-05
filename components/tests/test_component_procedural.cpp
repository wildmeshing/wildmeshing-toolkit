#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/procedural/procedural.hpp>
#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::utils;
using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_procedural_nocoord", "[components][procedural][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    // SECTION("grid2")
    {
        json component_json = {
            {"name", "grid2"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "diagonal"},
              {"dimensions", {5, 5}},
              {"cycles", {false, false}},
              {"coordinates", nullptr}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("grid3")
    {
        json component_json = {
            {"name", "grid3"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "freudenthal"},
              {"dimensions", {5, 5, 5}},
              {"cycles", {false, false, false}},
              {"coordinates", nullptr}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("triangle_fan")
    {
        json component_json = {
            {"name", "triangle_fan"},
            {"type", "fan"},
            {"fan", {{"size", 10}, {"coordinates", nullptr}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("disk")
    {
        json component_json = {
            {"type", "procedural"},
            {"name", "disk"},
            {"type", "disk"},
            {"disk", {{"size", 10}, {"coordinates", nullptr}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
}
TEST_CASE("component_procedural_coords", "[components][procedural][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    // SECTION("grid2")
    {
        json component_json = {
            {"name", "grid2"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "diagonal"},
              {"dimensions", {5, 5}},
              {"cycles", {false, false}},
              {"coordinates", {{"name", "vertices"}, {"spacing", {0.2, 0.2}}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("grid3")
    {
        json component_json = {
            {"name", "grid3"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "freudenthal"},
              {"dimensions", {5, 5, 5}},
              {"cycles", {false, false, false}},
              {"coordinates", {{"name", "vertices"}, {"spacing", {0.2, 0.2, 0.2}}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("triangle_fan")
    {
        json component_json = {
            {"name", "triangle_fan"},
            {"type", "fan"},
            {"fan",
             {{"size", 10},
              {"coordinates",
               {{"name", "vertices"},
                {"radius", 0.2},
                {"center", {.5, .5}},
                {"degrees", {0, 180}}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("disk")
    {
        json component_json = {
            {"name", "disk"},
            {"type", "disk"},
            {"disk",
             {{"size", 10},
              {"coordinates",
               {{"name", "vertices"},
                {"radius", 0.2},
                {"center", {.5, .5}},
                {"degree_offset", 0.0}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
}
TEST_CASE("component_procedural_cyclic_grids", "[components][procedural]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    // SECTION("grid2")
    {
        json component_json = {
            {"name", "grid2"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "diagonal"},
              {"dimensions", {5, 5}},
              {"cycles", {true, true}},
              {"coordinates", {{"name", "vertices"}, {"spacing", {0.2, 0.2}}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
    // SECTION("grid3")
    {
        json component_json = {
            {"name", "grid3"},
            {"type", "grid"},
            {"grid",
             {{"tiling", "freudenthal"},
              {"dimensions", {5, 5, 5}},
              {"cycles", {true, true, true}},
              {"coordinates", {{"name", "vertices"}, {"spacing", {0.2, 0.2, 0.2}}}}}}};


        CHECK_NOTHROW(wmtk::components::procedural(Paths(), component_json, cache));
    }
}
