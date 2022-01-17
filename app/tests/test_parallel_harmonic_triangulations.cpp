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

