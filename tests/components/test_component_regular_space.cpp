#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_regular_space", "[components][regular_space][.]")
{
    std::map<std::string, std::filesystem::path> files;
    std::map<std::string, long> tags_value;

    json regular_space_jason = {
        {"type", "regular space"},
        {"input", "inputdir"}, /*input dir*/
        {"output", "outputdir"}, /*output dir*/
        {"demension", 1}, /*0 for vertex, 1 for edge, 2 for face, 3 for tet*/
        {"tags_value", tags_value},
        {"split_tag_value"}};
}
