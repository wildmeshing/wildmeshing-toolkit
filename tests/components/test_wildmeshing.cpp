#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk_components/wildmeshing/wildmeshing.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("wildmeshing", "[components][wildmeshing][.]")
{
    json input = {
        {"planar", true},
        {"passes", 5},
        // {"input", data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh"},
        {"input", data_dir / "2d" / "rect1.msh"},
        {"target_edge_length", 0.05},
        {"intermediate_output", true},
        {"filename", "test"}};

    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::wildmeshing(input, files));
}
