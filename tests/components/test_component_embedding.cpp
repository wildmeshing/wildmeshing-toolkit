#include <igl/is_edge_manifold.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/embedding/embedding.hpp>
#include <wmtk_components/embedding/internal/Embedding.hpp>
#include <wmtk_components/embedding/internal/EmbeddingOptions.hpp>
#include <wmtk_components/input/input.hpp>

using json = nlohmann::json;
using namespace wmtk;
const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("embedding_function", "[components][embedding][2D]")
{
    // spdlog::warn("EdgeMesh has not been merged and used!");
    // EdgeMesh has not been merged
    std::map<std::string, std::filesystem::path> files;
    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 1},
        {"file", data_dir / "test_edgemesh.obj"}};
    wmtk::components::input(component_json, files);

    json mesh_embedding_json = {
        {"input_file", "file"},
        {"type", "embedding"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"tag_name", "vertices_tags"},
        {"input_tag_value", 1},
        {"embedding_tag_value", 0}};
    REQUIRE_NOTHROW(wmtk::components::embedding(mesh_embedding_json, files));
}