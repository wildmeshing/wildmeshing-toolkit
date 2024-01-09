#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/adaptive_tessellation/adaptive_tessellation.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk;
using namespace wmtk::tests;
const std::filesystem::path data_dir = WMTK_DATA_DIR;
TEST_CASE("at_test")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    nlohmann::json input_component_json = {
        {"name", "mesh"},
        {"file", (data_dir / "subdivided_quad.msh").string()},
        {"ignore_z", true}};

    wmtk::components::input(wmtk::components::base::Paths(), input_component_json, cache);

    nlohmann::json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"planar", true},
        {"target_edge_length", 0.1},
        {"intermediate_output", true},
        {"output", "test_maxinv"}};

    CHECK_NOTHROW(
        wmtk::components::adaptive_tessellation(wmtk::components::base::Paths(), input, cache));
}
// TODO add tests for adaptive tessellation