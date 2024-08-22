#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/multimesh.hpp>
#include <wmtk/io/Cache.hpp>

using namespace wmtk::components::base;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("multimesh", "[components][multimesh][.]")
{
    wmtk::io::Cache cache;

    json parent_json = {
        {"type", "input"},
        {"name", "parent"},
        // {"file", (data_dir / "bumpyDice.msh").string()},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), parent_json, cache);

    json child_json = {
        {"type", "input"},
        {"name", "child"},
        // {"file", (data_dir / "bumpyDice.msh").string()},
        {"ignore_z", true}};
    wmtk::components::input(Paths(), child_json, cache);

    json multimesh = {
        {"type", "multimesh"},
        {"parent", "parent"},
        {"child", "child"},
        {"output", "multimesh"}};

    wmtk::components::multimesh(Paths(), multimesh, cache);
}
