#include <catch2/catch_test_macros.hpp>

#include <igl/read_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <random>

#include <igl/doublearea.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include "HarmonicTet.hpp"
#include "spdlog/common.h"
#include "wmtk/utils/Delaunay.hpp"
#include "wmtk/utils/EnergyHarmonicTet.hpp"
#include "wmtk/utils/Logger.hpp"

#include <igl/Timer.h>
#include <igl/upsample.h>
#include <wmtk/utils/io.hpp>

using namespace wmtk;


TEST_CASE("harmonic-tet-energy", "[harmtri]")
{
    Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(4, 3);
    unit.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
    auto ee = wmtk::harmonic_energy(unit);
    REQUIRE(ee == 1.5);
    unit(0, 0) = -10;
    auto stretch = wmtk::harmonic_energy(unit);
    REQUIRE(stretch > 10);
}


auto stats = [](auto& har_tet) {
    auto total_e = 0.;
    auto cnt = 0;
    for (auto t : har_tet.get_tets()) {
        auto local_tuples = har_tet.oriented_tet_vertices(t);
        std::array<size_t, 4> local_verts;
        auto T = std::array<double, 12>();
        for (auto i = 0; i < 4; i++) {
            auto v = local_tuples[i].vid(har_tet);
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = har_tet.vertex_attrs[v].pos[j];
            }
        }
        auto e = wmtk::harmonic_tet_energy(T);
        total_e += e;
        cnt++;
    }
    return std::pair(total_e, cnt);
};

TEST_CASE("harmonic-tet-optim", "[harmtri]")
{
    auto vec_attrs = std::vector<Eigen::Vector3d>(4);
    vec_attrs[0] = Eigen::Vector3d(0, 0, 0);
    vec_attrs[1] = Eigen::Vector3d(1, 0, 0);
    vec_attrs[2] = Eigen::Vector3d(0, 1, 0);
    vec_attrs[3] = Eigen::Vector3d(0, 0, 1);
    auto tets = std::vector<std::array<size_t, 4>>{{{0, 1, 2, 3}}};
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets);

    // original energy 1.5
    har_tet.smooth_all_vertices();
    auto [E, cnt] = stats(har_tet);
    REQUIRE(
        E < 0.8); // Note: this may depend on the internal implementation detail of gradient descent
}

TEST_CASE("harmonic-tet-swaps", "[harmtri][.slow]")
{
    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    std::string filename = WMTK_DATA_DIR "/bunny.off";
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(filename, V, F);
        igl::upsample(V, F);
        wmtk::logger().info("Prepare to Delaunay: V {} F {}", V.rows(), F.rows());
        std::vector<wmtk::delaunay::Point3D> points(V.rows());
        for (auto i = 0; i < V.rows(); i++) {
            for (auto j = 0; j < 3; j++) points[i][j] = V(i, j);
        }
        auto [tet_V, tetT] = wmtk::delaunay::delaunay3D(points);
        vec_attrs.resize(tet_V.size());
        for (auto i = 0; i < tet_V.size(); i++) {
            for (auto j = 0; j < 3; j++) vec_attrs[i][j] = tet_V[i][j];
        }
        tets = tetT;
        wmtk::logger().info("Finishing Delaunay: V {} T {}", vec_attrs.size(), tets.size());
    }
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets);

    auto [E0, cnt0] = stats(har_tet);
    wmtk::logger().info("Start Energy E0  {} ", E0);
    har_tet.swap_all_edges();
    wmtk::logger().info("tet cap {}", har_tet.tet_capacity());
    har_tet.consolidate_mesh();
    wmtk::logger().info("tet cap {}", har_tet.tet_capacity());
    auto [E1, cnt1] = stats(har_tet);
    wmtk::logger().info(
        "Finishing Energy E1 {} E0 {} V {} T {}",
        E1,
        E0,
        har_tet.vert_capacity(),
        har_tet.tet_capacity());
    REQUIRE(E1 < E0);
}


TEST_CASE("parallel_harmonic-tet-swaps", "[parallel_harmtri][.slow]")
{
    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(WMTK_DATA_DIR "/Octocat.obj", V, F);
        std::vector<wmtk::delaunay::Point3D> points(V.rows());
        for (auto i = 0; i < V.rows(); i++) {
            for (auto j = 0; j < 3; j++) points[i][j] = V(i, j);
        }
        auto [tet_V, tetT] = wmtk::delaunay::delaunay3D(points);
        vec_attrs.resize(tet_V.size());
        for (auto i = 0; i < tet_V.size(); i++) {
            for (auto j = 0; j < 3; j++) vec_attrs[i][j] = tet_V[i][j];
        }
        tets = tetT;
    }

    double time;
    igl::Timer timer;
    for (int i = 1; i <= 32; i *= 2) {
        auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets, i);

        auto [E0, cnt0] = stats(har_tet);
        timer.start();
        har_tet.swap_all_edges(true);
        time = timer.getElapsedTimeInMilliSec();
        wmtk::logger().info("Time [{}]{}", i, time);
        har_tet.swap_all_faces();
        har_tet.consolidate_mesh();
        har_tet.smooth_all_vertices();
        auto [E1, cnt1] = stats(har_tet);
        REQUIRE(E1 < E0);
    }
}

#include <igl/writePLY.h>

TEST_CASE("gaussian-harmonic", "[.slow]")
{
    std::mt19937 gen;
    std::normal_distribution<> dist;

    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    {
        std::vector<wmtk::delaunay::Point3D> points(100000);
        for (auto i = 0; i < points.size(); i++) {
            for (auto j = 0; j < 3; j++) points[i][j] = dist(gen);
        }
        igl::writePLY(
            "gaussian_points_100k.ply",
            Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
                points[0].data(),
                points.size(),
                3),
            Eigen::MatrixXi::Zero(1, 3));
        auto [tet_V, tetT] = wmtk::delaunay::delaunay3D(points);
        vec_attrs.resize(tet_V.size());
        for (auto i = 0; i < tet_V.size(); i++) {
            for (auto j = 0; j < 3; j++) vec_attrs[i][j] = tet_V[i][j];
        }
        tets = tetT;
        wmtk::logger().info("Finishing Delaunay: V {} T {}", vec_attrs.size(), tets.size());
    }

    igl::Timer timer;
    double time;

    for (int i = 1; i <= 32; i *= 2) {
        wmtk::logger().info("Number of Threads: {}", i);
        auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets, i);
        timer.start();
        // auto [E0, cnt0] = stats(har_tet);
        // wmtk::logger().info("Start Energy E0  {} ", E0);
        har_tet.swap_all_edges(true);
        // har_tet.swap_all_faces(true);
        time = timer.getElapsedTimeInMilliSec();
        wmtk::logger().info("Time cost: {}", time / 1e3);
        har_tet.consolidate_mesh();
        wmtk::logger().info("tet cap {}", har_tet.tet_capacity());
    }
}
