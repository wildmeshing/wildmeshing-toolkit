#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>

#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"

#include <Eigen/Core>

TEST_CASE("separate-manifold-patch", "[test_util]")
{
    std::vector<Eigen::Vector3d> vertices = {
        {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(0, -1, 0)}};
    std::vector<std::array<size_t, 3>> faces = {{
        {{0, 1, 2}},
        {{0, 1, 3}},
        {{0, 1, 4}},
    }};

    std::vector<Eigen::Vector3d> out_v;
    std::vector<std::array<size_t, 3>> out_f;
    wmtk::separate_to_manifold(vertices, faces, out_v, out_f);
    REQUIRE(out_v.size() == 9);
    REQUIRE(out_f.size() == 3);
}