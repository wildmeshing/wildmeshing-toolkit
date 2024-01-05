#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/wildmeshing/wildmeshing.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("wildmeshing", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"type", "input"},
        {"name", "mesh"},
        {"file", data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh"},
        // {"input", data_dir / "2d" / "rect1.msh"},
        {"ignore_z", true}};
    wmtk::components::input(input_component_json, cache);

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.01},
        {"intermediate_output", true},
        {"output", "test"}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(input, cache));
}
