#include <random>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk_components/delaunay/delaunay.hpp>
#include <wmtk_components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk_components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/output/output.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_delaunay", "[components][delaunay][.]")
{
    std::map<std::string, std::filesystem::path> files;

    // input
    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 0},
            {"file", data_dir / "bunny.off"}};
        wmtk::components::input(input_component_json, files);
    }

    json component_json = {
        {"type", "input"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"cell_dimension", 3}};

    CHECK_NOTHROW(wmtk::components::delaunay(component_json, files));

    {
        json component_json = {
            {"type", "output"},
            {"input", "output_mesh"},
            {"cell_dimension", 3},
            {"file", "component_delaunay_3d"}};

        CHECK_NOTHROW(wmtk::components::output(component_json, files));
    }
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
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_2d_five_points.vtu", vertices, faces);
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

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_2d_random.vtu", vertices, faces);
    }
}

TEST_CASE("delaunay_3d_nine_points", "[components][delaunay][.]")
{
    // create points
    std::vector<Eigen::Vector3d> points;
    points.push_back({-1, -1, -1});
    points.push_back({1, -1, -1});
    points.push_back({-1, 1, -1});
    points.push_back({-1, -1, 1});
    points.push_back({1, 1, -1});
    points.push_back({-1, 1, 1});
    points.push_back({1, -1, 1});
    points.push_back({1, 1, 1});
    points.push_back({0, 0, 0});

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;

    CHECK_NOTHROW(wmtk::components::internal::delaunay_3d(points, vertices, faces));

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_3d_nine_points.vtu", vertices, faces);
    }
}

TEST_CASE("delaunay_3d_random", "[components][delaunay][.]")
{
    std::uniform_real_distribution<double> distribution(-1, 1);
    std::default_random_engine random_engine;

    // create points
    std::vector<Eigen::Vector3d> points;
    points.reserve(100);
    for (size_t i = 0; i < 100; ++i) {
        const double x = distribution(random_engine);
        const double y = distribution(random_engine);
        const double z = distribution(random_engine);
        points.push_back({x, y, z});
    }

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;

    CHECK_NOTHROW(wmtk::components::internal::delaunay_3d(points, vertices, faces));

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_3d_random.vtu", vertices, faces);
    }
}