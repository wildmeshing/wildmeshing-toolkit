#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/wildmeshing/wildmeshing.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::base;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("wildmeshing", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        {"file", data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh"},
        // {"input", data_dir / "2d" / "rect1.msh"},
        {"ignore_z", true}};
    wmtk::components::input(Paths(), input_component_json, cache);

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.01},
        {"intermediate_output", true},
        {"output", "test"}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}


TEST_CASE("wildmeshing_3d", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        // {"input", data_dir / "sphere_coarse_.msh"},
        // {"input", data_dir / "tet.msh"},
        {"input", data_dir / "sphere_coarse_005_.msh"}};
    wmtk::components::input(Paths(), input_component_json, cache);

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.1},
        {"intermediate_output", true},
        {"output", "test_maxinv"}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}

TEST_CASE("wildmeshing_3d_multimesh", "[components][wildmeshing][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json input_component_json = {
        {"name", "mesh"},
        // {"input", data_dir / "sphere_coarse_.msh"},
        // {"input", data_dir / "tet.msh"},
        {"file", data_dir / "sphere_coarse_005_.msh"},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), input_component_json, cache);

    json attributes = {{"position", "vertices"}};

    json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"target_edge_length", 0.1},
        {"intermediate_output", true},
        {"output", "test_multimesh"},
        {"track_boundary_child_mesh", false},
        {"pass_through", std::vector<int64_t>()},
        {"attributes", attributes}};


    CHECK_NOTHROW(wmtk::components::wildmeshing(Paths(), input, cache));
}