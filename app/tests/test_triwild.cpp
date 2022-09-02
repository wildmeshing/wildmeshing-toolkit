
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
    // Test loading
    std::string input_path = WMT_DATA_DIR "/2d/4triangles.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(input_path, V, F);

    triwild::TriWild triwild;
    triwild.create_mesh(V,F);

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(triwild.vertex_attrs.size() == 5);
}

TEST_CASE("triwild2", "[triwild_int]")
{
    // Test smoothing
    std::string input_path = WMT_DATA_DIR "/2d/4triangles.obj";

    Eigen::MatrixXd V,V2;
    Eigen::MatrixXi F,F2;
    igl::read_triangle_mesh(input_path, V, F);

    triwild::TriWild triwild;
    triwild.create_mesh(V,F);

    triwild.smooth_all_vertices();

    triwild.export_mesh(V2,F2);

    REQUIRE(triwild.check_mesh_connectivity_validity());
    REQUIRE(F2.rows() == F.rows());
    
}
