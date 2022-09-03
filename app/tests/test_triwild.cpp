
#include <TriWild.h>
#include <common.h>

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/io.hpp>
#include "spdlog/common.h"

using namespace wmtk;
using namespace triwild;

TEST_CASE("triwild1", "[triwild_int]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // igl::read_triangle_mesh(input_path, V, F);

    Eigen::MatrixXd V(5,2);
    V <<-1,1,
        1,1,
        -1,-1,
        1,-1,
        0,0;

    Eigen::MatrixXi F(4,3);
    F <<
    2,0,4,
    1,3,4,
    2,4,3,
    0,1,4;

    triwild::TriWild triwild;
    triwild.create_mesh(V,F);

    triwild.write_obj("4triangles.obj");

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(triwild.vertex_attrs.size() == 5);

}

TEST_CASE("triwild2", "[triwild_int]")
{
    // tests smoothing on single triangle

    Eigen::MatrixXd V(3,2),V2;
    V <<-1,0,
        1,0,
        0,0.1;
    
    // V.col(1) = - V.col(1);

    Eigen::MatrixXi F(1,3),F2;
    F << 0,1,2;

    triwild::TriWild triwild;
    triwild.create_mesh(V,F);


    triwild.write_obj("01-before.obj");
    triwild.smooth_all_vertices();
    triwild.write_obj("02-after.obj");

    triwild.export_mesh(V2,F2);

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}


TEST_CASE("triwild3", "[triwild_int]")
{
    // tests smoothing

    Eigen::MatrixXd V(5,2),V2;
    V <<-1,1,
        1,1,
        -1,-1,
        1,-1,
        0,0.5;
    
    V.col(1) = - V.col(1);

    Eigen::MatrixXi F(4,3),F2;
    F <<
    2,0,4,
    1,3,4,
    2,4,3,
    0,1,4;

    triwild::TriWild triwild;
    triwild.create_mesh(V,F);


    triwild.write_obj("01-before.obj");
    for (unsigned i=0; i<10; ++i)
        triwild.smooth_all_vertices();
    triwild.write_obj("02-after.obj");

    triwild.export_mesh(V2,F2);

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}
