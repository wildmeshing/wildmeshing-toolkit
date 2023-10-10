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
    wmtk::RowVectors2d points(5, 2);
    points.row(0) << -1, -1;
    points.row(1) << -1, 1;
    points.row(2) << 1, -1;
    points.row(3) << 1, 1;
    points.row(4) << 0, 0;

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    CHECK_NOTHROW(std::tie(vertices, faces) = wmtk::components::internal::delaunay_2d(points));
    CHECK(points == vertices);

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
    wmtk::RowVectors2d points(100, 2);
    for (size_t i = 0; i < 100; ++i) {
        const double x = distribution(random_engine);
        const double y = distribution(random_engine);
        points.row(i) << x, y;
    }

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    CHECK_NOTHROW(std::tie(vertices, faces) = wmtk::components::internal::delaunay_2d(points));
    CHECK(points == vertices);

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_2d_random.vtu", vertices, faces);
    }
}

TEST_CASE("delaunay_3d_nine_points", "[components][delaunay][.]")
{
    // create points
    wmtk::RowVectors3d points(9, 3);
    points.row(0) << -1, -1, -1;
    points.row(1) << 1, -1, -1;
    points.row(2) << -1, 1, -1;
    points.row(3) << -1, -1, 1;
    points.row(4) << 1, 1, -1;
    points.row(5) << -1, 1, 1;
    points.row(6) << 1, -1, 1;
    points.row(7) << 1, 1, 1;
    points.row(8) << 0, 0, 0;

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    CHECK_NOTHROW(std::tie(vertices, faces) = wmtk::components::internal::delaunay_3d(points));
    CHECK(points == vertices);

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
    wmtk::RowVectors3d points(100, 3);
    for (size_t i = 0; i < 100; ++i) {
        const double x = distribution(random_engine);
        const double y = distribution(random_engine);
        const double z = distribution(random_engine);
        points.row(i) << x, y, z;
    }

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    CHECK_NOTHROW(std::tie(vertices, faces) = wmtk::components::internal::delaunay_3d(points));
    CHECK(points == vertices);

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_3d_random.vtu", vertices, faces);
    }
}
