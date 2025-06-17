#include <IncrementalTetWild.h>
#include <common.h>
#include <wmtk/TetMesh.h>

// #include <igl/copyleft/cgal/remesh_intersections.h>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/utils/AMIPS.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/io.hpp>
#include "spdlog/common.h"


using namespace wmtk;
using namespace tetwild;

TEST_CASE("topo_test", "[topology preservation]")
{
    REQUIRE(1 == 1);
}

TEST_CASE("cgal_check", "[topology preservation]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // igl::read_triangle_mesh(
    //     "/home/jiacheng/jiacheng/incremental_tetwild/wildmeshing-toolkit/build/"
    //     "random_triangles.obj",
    //     V,
    //     F);
    // std::cout << V.rows() << std::endl;
    // std::cout << F.rows() << std::endl;
    REQUIRE(1 == 1);
}