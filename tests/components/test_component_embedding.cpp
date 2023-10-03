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

// this data is the one Prototype displayed
// void generateRandomData(Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges, int vertices_num = 10)
// {
//     vertices.resize(vertices_num, 2);
//     edges.resize(vertices_num, 2);
//     std::srand((unsigned)time(NULL));
//     int intmax = std::numeric_limits<double>::lowest();
//     for (int i = 0; i < vertices_num; ++i) {
//         //rand()/double(RAND_MAX)
//         vertices(i, 0) = (std::rand() / double(intmax) - 0.5) * 10;
//         vertices(i, 1) = (std::rand() / double(intmax) - 0.5) * 10;
//         edges(i, 0) = i;
//         edges(i, 1) = (i + 1) % (vertices_num);
//     }
// }

// void generateDatawithSmallTriangles(Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges)
// {
//     vertices.resize(10, 2);
//     edges.resize(10, 2);
//     vertices << 3.68892, 4.69909, 3.72799, -2.02109, 3.66695, -1.08875, -1.64113, 3.15912,
//     -1.72872,
//         2.57591, 1.20228, 4.31211, 2.19108, -0.595569, 2.78527, -0.139927, -4.85534, 0.28489,
//         -4.68596, 3.57936;
//     edges << 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 0;
// }

TEST_CASE("embedding_function", "[components][embedding][2D]")
{
    // spdlog::warn("EdgeMesh has not been merged and used!");
    // EdgeMesh has not been merged
    std::map<std::string, std::filesystem::path> files;
    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 1},
        {"file", data_dir / "edge_test.obj"}};
    wmtk::components::input(component_json, files);
    //files["input_mesh"] = data_dir / "edge_test.obj";

    json mesh_embedding_json = {
        {"input_file", "file"},
        {"type", "embedding"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"tag_name", "vertices_tags"},
        {"input_tag_value", 1},
        {"embedding_tag_value", 0},
        {"offset_tag_value", 2},
        {"resolute_level", 0}};
    REQUIRE_NOTHROW(wmtk::components::embedding(mesh_embedding_json, files));
}