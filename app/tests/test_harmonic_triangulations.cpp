#include <catch2/catch.hpp>

#include <igl/read_triangle_mesh.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/TetMesh.h>

#include <igl/doublearea.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include "HarmonicTet.hpp"
#include "spdlog/common.h"
#include "wmtk/utils/Delaunay.hpp"
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;


TEST_CASE("harmonic-tet-energy")
{
    Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(4, 3);
    unit.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
    auto ee = wmtk::harmonic_energy(unit);
    REQUIRE(ee == 1.5);
    unit(0, 0) = -10;
    auto stretch = wmtk::harmonic_energy(unit);
    REQUIRE(stretch > 10);
}

TEST_CASE("harmonic-tet-swaps")
{
    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(WMT_DATA_DIR "/sphere.obj", V, F);
        std::vector<wmtk::Point3D> points(V.rows());
        for (auto i = 0; i < V.rows(); i++) {
            for (auto j = 0; j < 3; j++) points[i][j] = V(i, j);
        }
        auto [tet_V, tetT] = wmtk::delaunay3D(points);
        vec_attrs.resize(tet_V.size());
        for (auto i = 0; i < tet_V.size(); i++) {
            for (auto j = 0; j < 3; j++) vec_attrs[i][j] = tet_V[i][j];
        }
        tets = tetT;
        REQUIRE(tetT.size() == 1262);
    }
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets);
    har_tet.swap_all_edges();
    har_tet.consolidate_mesh();
    REQUIRE(har_tet.tet_capacity() < 1200);
}