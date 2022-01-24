#include <igl/read_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>

#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"

#include <Eigen/Core>
#include <igl/is_edge_manifold.h>

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

TEST_CASE("manifold-separate-test-37989", "[test_util]")
{
    std::string filename = WMT_DATA_DIR "/37989_sf.obj";
    Eigen::MatrixXd V, outV;
    Eigen::MatrixXi F, outF;
    igl::read_triangle_mesh(filename, V,F);
    std::vector<Eigen::Vector3d> out_v;
    std::vector<std::array<size_t, 3>> out_f;

    std::vector<Eigen::Vector3d> vertices(V.rows());
    std::vector<std::array<size_t, 3>> faces(F.rows());
    for (auto i=0; i<V.rows(); i++) vertices[i] = V.row(i);
    for (auto i=0; i<F.rows(); i++) faces[i] = {{(size_t)F(i,0), (size_t)F(i,1), (size_t)F(i,2)}};

    wmtk::separate_to_manifold(vertices, faces, out_v, out_f);

    outV.resize(out_v.size(), 3);
    outF.resize(out_f.size(), 3);
    for (auto i=0; i<out_v.size(); i++) {
        outV.row(i) = out_v[i];
    }
    for (auto i=0; i<out_f.size(); i++) {
        outF.row(i) << out_f[i][0],out_f[i][1],out_f[i][2];
    }
    REQUIRE(igl::is_edge_manifold(outF));
}