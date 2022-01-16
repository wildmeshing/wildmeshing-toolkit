#include <catch2/catch.hpp>

#include <igl/read_triangle_mesh.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/TetMesh.h>

#include <igl/doublearea.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include "ParallelHarmonicTet.hpp"
#include "spdlog/common.h"
#include "wmtk/utils/Delaunay.hpp"
#include "wmtk/utils/EnergyHarmonicTet.hpp"
#include "wmtk/utils/Logger.hpp"

#include <tbb/concurrent_vector.h>

#include <igl/Timer.h>

using namespace wmtk;

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
                T[i * 3 + j] = har_tet.m_vertex_attribute[v][j];
            }
        }
        auto e = wmtk::harmonic_tet_energy(T);
        total_e += e;
        cnt++;
    }
    return std::pair(total_e, cnt);
};


TEST_CASE("parallel_harmonic-tet-swaps", "[parallel_harmtri]")
{
    auto vec_attrs = tbb::concurrent_vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(WMT_DATA_DIR "/octocat.obj", V, F);
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
        // REQUIRE(tetT.size() == 1262);
    }

    double time;
    igl::Timer timer;
    for (int i = 1; i <= 4; i *= 2) {
        auto har_tet = harmonic_tet::ParallelHarmonicTet(vec_attrs, tets, i);

        auto [E0, cnt0] = stats(har_tet);
        timer.start();
        har_tet.swap_all_edges();
        time = timer.getElapsedTimeInMilliSec();
        std::cout << time << std::endl;
        // har_tet.swap_all_faces();
        // har_tet.consolidate_mesh();
        // har_tet.smooth_all_vertices();
        // auto [E1, cnt1] = stats(har_tet);
        // REQUIRE(E1 / cnt1 < E0 / cnt0);
    }
}