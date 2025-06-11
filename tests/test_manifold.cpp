#include <igl/read_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
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
    std::vector<size_t> freeze_v;
    wmtk::separate_to_manifold(vertices, faces, out_v, out_f, freeze_v);
    REQUIRE(out_v.size() == 9);
    REQUIRE(out_f.size() == 3);
}

TEST_CASE("manifold-separate-test-37989", "[test_util]")
{
    std::string filename = WMTK_DATA_DIR "/37989_sf.obj";
    wmtk::manifold_internal::Vertices V;
    wmtk::manifold_internal::Facets F;
    igl::read_triangle_mesh(filename, V, F);
    REQUIRE_FALSE(igl::is_edge_manifold(F));
    std::vector<size_t> modified_vertices;
    wmtk::manifold_internal::resolve_nonmanifoldness(V, F, modified_vertices);
    REQUIRE(modified_vertices.size() > 0);
    REQUIRE(igl::is_edge_manifold(F));
    Eigen::VectorXi VI;
    REQUIRE(igl::is_vertex_manifold(F, VI));
}
