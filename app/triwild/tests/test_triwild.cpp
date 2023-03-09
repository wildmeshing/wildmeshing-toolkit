
#include <TriWild.h>

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/io.hpp>
#include "spdlog/common.h"

using namespace wmtk;
using namespace triwild;

TEST_CASE("triwild_load", "[triwild_int]")
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

TEST_CASE("triwild_smooth_1triangle", "[triwild_int]")
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

    double quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality > 5);

    //triwild.write_obj("01-before.obj");
    triwild.smooth_all_vertices();
    //triwild.write_obj("02-after.obj");

    triwild.export_mesh(V2,F2);

    quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality < 2.1);

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}


TEST_CASE("triwild_smooth_4triangles", "[triwild_int]")
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

    double quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality > 2.5);


    // triwild.write_obj("01-before.obj");
    for (unsigned i=0; i<10; ++i)
        triwild.smooth_all_vertices();
    // triwild.write_obj("02-after.obj");

    triwild.export_mesh(V2,F2);

    quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality < 2.5);


    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}


TEST_CASE("triwild_boundary", "[triwild_int]")
{
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

    double quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality > 2.5);

    // detect the boundary and freeze the vertices on it
    triwild.for_each_vertex(
        [&](auto& v)
        {
            triwild.vertex_attrs[v.vid(triwild)].fixed = triwild.is_boundary_vertex(v);
        }
    );

    REQUIRE(triwild.vertex_attrs[0].fixed == true);
    REQUIRE(triwild.vertex_attrs[1].fixed == true);
    REQUIRE(triwild.vertex_attrs[2].fixed == true);
    REQUIRE(triwild.vertex_attrs[3].fixed == true);
    REQUIRE(triwild.vertex_attrs[4].fixed == false);

    for (unsigned i=0; i<10; ++i)
        triwild.smooth_all_vertices();

    triwild.export_mesh(V2,F2);

    REQUIRE(V2.row(0) == V.row(0));
    REQUIRE(V2.row(1) == V.row(1));
    REQUIRE(V2.row(2) == V.row(2));
    REQUIRE(V2.row(3) == V.row(3));
    REQUIRE(V2.row(4) != V.row(4));

    quality = triwild.get_quality_all_triangles().mean();
    REQUIRE(quality < 2.5);


    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}
