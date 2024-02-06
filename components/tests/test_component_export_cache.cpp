#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/export_cache/export_cache.hpp>
#include <wmtk/components/input/input.hpp>

using namespace wmtk::components::base;
using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_export_cache", "[components][export_cache]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    {
        const std::filesystem::path input_file = data_dir / "small.msh";
        json component_json = {
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false}};


        CHECK_NOTHROW(wmtk::components::input(Paths(), component_json, cache));
    }

    wmtk::io::Cache cache_dump("wmtk_dump", ".");

    // export cache
    {
        json o;
        o["folder"] = cache_dump.get_cache_path() / "exported_cache";

        CHECK_NOTHROW(wmtk::components::export_cache(Paths(), o, cache));
    }
}