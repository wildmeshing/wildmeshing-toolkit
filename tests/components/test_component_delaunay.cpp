#include <random>

#include <igl/writeOBJ.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/delaunay/delaunay.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_delaunay", "[components][delaunay][.]")
{
    // TODO read input first
    // ...

    json component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"output", "output_mesh"},
        {"dimension", 2}};

    std::map<std::string, std::filesystem::path> files;

    CHECK_NOTHROW(wmtk::components::delaunay(component_json, files));
}

TEST_CASE("delaunay_2d_five_points", "[components][delaunay]")
{
    // create points
    std::vector<Eigen::Vector2d> points;
    points.push_back({-1, -1});
    points.push_back({-1, 1});
    points.push_back({1, -1});
    points.push_back({1, 1});
    points.push_back({0, 0});

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;

    CHECK_NOTHROW(wmtk::components::internal::delaunay_2d(points, vertices, faces));

    if (false) {
        Eigen::MatrixXd V3;
        V3.resize(vertices.rows(), vertices.cols() + 1);
        V3.setZero();
        V3.block(0, 0, vertices.rows(), vertices.cols()) = vertices;
        igl::writeOBJ("delaunay_2d_five_points.obj", V3, faces);
    }
}

TEST_CASE("delaunay_2d_random", "[components][delaunay]")
{
    std::uniform_real_distribution<double> distribution(-1, 1);
    std::default_random_engine random_engine;

    // create points
    std::vector<Eigen::Vector2d> points;
    points.reserve(100);
    for (size_t i = 0; i < 100; ++i) {
        const double x = distribution(random_engine);
        const double y = distribution(random_engine);
        points.push_back({x, y});
    }

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;

    CHECK_NOTHROW(wmtk::components::internal::delaunay_2d(points, vertices, faces));

    {
        Eigen::MatrixXd V3;
        V3.resize(vertices.rows(), vertices.cols() + 1);
        V3.setZero();
        V3.block(0, 0, vertices.rows(), vertices.cols()) = vertices;
        igl::writeOBJ("delaunay_2d_random.obj", V3, faces);
    }
}