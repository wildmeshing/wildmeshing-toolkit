#include <igl/is_edge_manifold.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/embedding/embedding.hpp>
#include <wmtk_components/embedding/internal/Embedding.hpp>
#include <wmtk_components/embedding/internal/EmbeddingOptions.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/isosurface_extraction/internal/IsosurfaceExtraction.hpp>
#include <wmtk_components/isosurface_extraction/internal/IsosurfaceExtractionOptions.hpp>
#include <wmtk_components/isosurface_extraction/isosurface_extraction.hpp>

using json = nlohmann::json;
using namespace wmtk;
const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("extraction_function", "[components][extraction][2D]")
{
    // spdlog::warn("EdgeMesh has not been merged and used!");
    // EdgeMesh has not been merged
    std::map<std::string, std::filesystem::path> files;

    json mesh_extraction = {
        {"type", "extraction"},
        {"input", "input"},
        {"output", "result"},
        {"scalar_field_tag_type", "long"},
        {"iso_value", 0.5},
        {"inflate_abs", 0.5},
        {"inflate_rel", 0.5},
        {"resolution_rate", -1},
        {"input_tag_value", 0},
        {"embedding_tag_value", 1},
        {"offset_tag_value", 2},
        {"iteration_times", 5},
        {"lock_boundary", true}};
    files["input"] = data_dir / "embedding_result.hdf5";
    wmtk::components::isosurface_extraction(mesh_extraction, files);
}