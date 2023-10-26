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
    json mesh_embedding_json = {
        {"input_file", "file"},
        {"type", "embedding"},
        {"input", data_dir / "test_edgemesh.hdf5"},
        {"output", "output_mesh"},
        {"tag_name", "vertices_tags"},
        {"input_tag_value", 1},
        {"embedding_tag_value", 0},
        {"input_dimension", 1},
        {"output_dimension", 2}};
    REQUIRE_NOTHROW(wmtk::components::embedding(mesh_embedding_json, files));
}