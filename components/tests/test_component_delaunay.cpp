#include <random>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/components/delaunay/delaunay.hpp>
#include <wmtk/components/delaunay/internal/delaunay_2d.hpp>
#include <wmtk/components/delaunay/internal/delaunay_3d.hpp>
#include <wmtk/components/delaunay/internal/delaunay_geogram.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = "../data/";

// all rows in p appear also in v
void check_p_is_contained_in_v(
    const Eigen::Ref<Eigen::MatrixXd> p,
    const Eigen::Ref<Eigen::MatrixXd> v)
{
    REQUIRE(p.cols() == v.cols());
    REQUIRE(p.rows() <= v.rows());

    std::vector<Eigen::VectorXd> vv;
    vv.reserve(v.rows());
    for (Eigen::Index i = 0; i < v.rows(); ++i) {
        vv.emplace_back(v.row(i));
    }

    auto v_less = [](const Eigen::Ref<Eigen::VectorXd> a, const Eigen::Ref<Eigen::VectorXd> b) {
        for (Eigen::Index i = 0; i < a.rows() - 1; ++i) {
            if (a[i] != b[i]) {
                return a[i] < b[i];
            }
        }
        return a[a.rows() - 1] < b[b.rows() - 1];
    };

    std::sort(vv.begin(), vv.end(), v_less);
    for (Eigen::Index i = 0; i < p.rows(); ++i) {
        const Eigen::VectorXd r = p.row(i);
        CHECK(std::find(vv.begin(), vv.end(), r) != vv.end());
    }
}

// TEST_CASE("component_delaunay", "[components][delaunay]")
// {
//     std::map<std::string, std::filesystem::path> files;

//     // input
//     {
//         json input_component_json = {
//             {"type", "input"},
//             {"name", "input_mesh_test_component_delaunay"},
//             {"cell_dimension", 0},
//             {"file", data_dir / "piece_0.msh"}};
//         wmtk::components::input(input_component_json, files);
//     }

//     json component_json = {
//         {"type", "input"},
//         {"input", "input_mesh_test_component_delaunay"},
//         {"output", "output_mesh_test_component_delaunay"},
//         {"cell_dimension", 3}};

//     CHECK_NOTHROW(wmtk::components::delaunay(component_json, files));

//     //{
//     //    json component_json = {
//     //        {"type", "output"},
//     //        {"input", "output_mesh"},
//     //        {"cell_dimension", 3},
//     //        {"file", "component_delaunay_3d"}};
//     //
//     //    CHECK_NOTHROW(wmtk::components::output(component_json, files));
//     //}
// }

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
    check_p_is_contained_in_v(points, vertices);

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_2d_random.vtu", vertices, faces);
    }
}

TEST_CASE("delaunay_3d_nine_points", "[components][delaunay]")
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

TEST_CASE("delaunay_3d_random", "[components][delaunay]")
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
    check_p_is_contained_in_v(points, vertices);

    if (false) {
        paraviewo::VTUWriter writer;
        writer.write_mesh("delaunay_3d_random.vtu", vertices, faces);
    }
}

TEST_CASE("delaunay_throw", "[components][delaunay]")
{
    Eigen::MatrixXd points;
    SECTION("0d")
    {
        points.resize(1, 1);
        points.row(0) << 0;
    }
    SECTION("4d")
    {
        points.resize(1, 4);
        points.row(0) << 0, 1, 2, 3;
    }

    CHECK_THROWS(wmtk::components::internal::delaunay_geogram(points));
}

TEST_CASE("delaunay_empty_points", "[components][delaunay]")
{
    Eigen::MatrixXd points;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    CHECK_NOTHROW(std::tie(vertices, faces) = wmtk::components::internal::delaunay_geogram(points));

    CHECK(vertices.rows() == 0);
    CHECK(faces.rows() == 0);
}