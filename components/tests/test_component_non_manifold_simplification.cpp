#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/non_manifold_input/non_manifold_input.hpp>
#include <wmtk/components/non_manifold_simplification/non_manifold_simplification.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>

using namespace wmtk::components::base;
using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;


TEST_CASE("component_non_manifold_simplification", "[components][non_manifold_simplification]")
{
    logger().set_level(spdlog::level::off);

    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    {
        const std::filesystem::path input_file = data_dir / "hour_glass.msh";
        json j = {
            {"type", "non_manifold_input"},
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()},
            {"non_manifold_vertex_label", "nmv"},
            {"non_manifold_edge_label", "nme"},
            {"non_manifold_tag_value", 1}};

        REQUIRE_NOTHROW(wmtk::components::non_manifold_input(Paths(), j, cache));
    }

    // simplification
    {
        json j = {
            {"type", "non_manifold_simplification"},
            {"input", "input_mesh"},
            {"output", "output_mesh"},
            {"position", "vertices"},
            {"iterations", 2},
            {"length_abs", -1},
            {"length_rel", 10},
            {"envelope_size", 0.001},
            {"non_manifold_vertex_label", "nmv"},
            {"non_manifold_edge_label", "nme"},
            {"non_manifold_tag_value", 1},
            {"pass_through", json::array()}};

        REQUIRE_NOTHROW(wmtk::components::non_manifold_simplification(Paths(), j, cache));
    }
}