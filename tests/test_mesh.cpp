#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>


TEST_CASE("test_mesh_virtuals", "[mesh]")
{
    wmtk::PointMesh pm;
    wmtk::EdgeMesh em;
    wmtk::TriMesh fm;
    wmtk::TetMesh tm;

    REQUIRE(pm.top_cell_dimension() == 0);
    REQUIRE(em.top_cell_dimension() == 1);
    REQUIRE(fm.top_cell_dimension() == 2);
    REQUIRE(tm.top_cell_dimension() == 3);

    REQUIRE(pm.top_simplex_type() == wmtk::PrimitiveType::Vertex);
    REQUIRE(em.top_simplex_type() == wmtk::PrimitiveType::Edge);
    REQUIRE(fm.top_simplex_type() == wmtk::PrimitiveType::Face);
    REQUIRE(tm.top_simplex_type() == wmtk::PrimitiveType::Tetrahedron);
}
