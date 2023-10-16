#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/EdgeMeshReader.hpp>
#include <wmtk_components/input/input.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_input", "[components][input][.]")
{
    SECTION("should pass")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", data_dir / "bunny.off"}};

        std::map<std::string, std::filesystem::path> files;

        CHECK_NOTHROW(wmtk::components::input(component_json, files));
    }

    SECTION("should throw")
    {
        json component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", "In case you ever name your file like that: What is wrong with you?"}};

        std::map<std::string, std::filesystem::path> files;
        CHECK_THROWS(wmtk::components::input(component_json, files));
    }
}

TEST_CASE("component_input_point", "[components][input][.]")
{
    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 0},
        {"file", data_dir / "bunny_points.obj"}};

    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::input(component_json, files));
}

TEST_CASE("component_input_edge", "[components][input][.]")
{
    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"cell_dimension", 1},
        {"file", data_dir / "test_edgemesh.obj"}};
    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::input(component_json, files));
}

TEST_CASE("edgemesh_reader", "[component][input][io][.]")
{
    using namespace wmtk;
    Eigen::Matrix<long, -1, -1> edges;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXd vertices_w;
    Eigen::MatrixXd vertices_texture;
    Eigen::MatrixXd vertices_normal;
    Eigen::MatrixXd vertices_parameter;
    SECTION("should fail -- unknown files")
    {
        EdgeMeshReader reader("this is a invalid file!", EdgeMeshReader::OBJ);
        CHECK_THROWS(reader.read(
            edges,
            vertices,
            vertices_w,
            vertices_texture,
            vertices_normal,
            vertices_parameter));
    }
    SECTION("should fail -- off is not supported for now")
    {
        EdgeMeshReader reader(data_dir / "test_edgemesh.obj", EdgeMeshReader::OFF);
        CHECK_THROWS(reader.read(
            edges,
            vertices,
            vertices_w,
            vertices_texture,
            vertices_normal,
            vertices_parameter));
    }
    SECTION("should fail -- off is not supported for now")
    {
        EdgeMeshReader reader(data_dir / "test_edgemesh.obj", EdgeMeshReader::OBJ);
        CHECK_NOTHROW(reader.read(
            edges,
            vertices,
            vertices_w,
            vertices_texture,
            vertices_normal,
            vertices_parameter));
        REQUIRE(edges.rows() == 10);
        REQUIRE(vertices.rows() == 10);
        REQUIRE(vertices_w.rows() == 10);
        REQUIRE(vertices_texture.rows() == 10);
        REQUIRE(vertices_normal.rows() == 10);
        REQUIRE(vertices_parameter.rows() == 10);

        for (long i = 0; i < edges.rows(); ++i) {
            spdlog::info("{}: {},{}", i, edges(i, 0), edges(i, 1));
        }
        for (long i = 0; i < vertices.rows(); ++i) {
            spdlog::info(
                "{}: {},{},{},{}",
                i,
                vertices(i, 0),
                vertices(i, 1),
                vertices(i, 2),
                vertices_w(i, 0));
        }
        for (long i = 0; i < vertices_texture.rows(); ++i) {
            spdlog::info(
                "{}: {},{},[{}]",
                i,
                vertices_texture(i, 0),
                vertices_texture(i, 1),
                vertices_texture(i, 2));
        }
        for (long i = 0; i < vertices_normal.rows(); ++i) {
            spdlog::info(
                "{}: {},{},{}",
                i,
                vertices_normal(i, 0),
                vertices_normal(i, 1),
                vertices_normal(i, 2));
        }
        for (long i = 0; i < vertices_parameter.rows(); ++i) {
            spdlog::info(
                "{}: {},{},{}",
                i,
                vertices_parameter(i, 0),
                vertices_parameter(i, 1),
                vertices_parameter(i, 2));
        }
    }
}