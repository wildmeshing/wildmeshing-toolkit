#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/tuples_preserving_primitive_types.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace tests;
using namespace tests_3d;

TEST_CASE("simplex_tuples_preserving_primitive_types", "[simplex_collection]")
{
    SECTION("trimesh_tri_vertex")
    {
        DEBUG_TriMesh m = single_triangle();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Vertex);

        REQUIRE(fci.size() == 2);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Triangle) == 0);
        }
    }
    SECTION("trimesh_tri_edge")
    {
        DEBUG_TriMesh m = single_triangle();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Edge);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Triangle) == 0);
        }
    }
    SECTION("trimesh_tri_tri")
    {
        DEBUG_TriMesh m = single_triangle();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Triangle);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Triangle) == 0);
        }
    }
    SECTION("tetmesh_tet_vertex")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Tetrahedron,
            PrimitiveType::Vertex);

        REQUIRE(fci.size() == 3);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_tri_vertex")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Vertex);

        REQUIRE(fci.size() == 2);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_edge_vertex")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Edge,
            PrimitiveType::Vertex);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_vertex_vertex")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Vertex,
            PrimitiveType::Vertex);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_tet_edge")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Tetrahedron,
            PrimitiveType::Edge);

        REQUIRE(fci.size() == 2);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_tri_edge")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Edge);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_tet_tri")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Tetrahedron,
            PrimitiveType::Triangle);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
    SECTION("tetmesh_tri_tri")
    {
        DEBUG_TetMesh m = single_tet();

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

        const auto fci = simplex::tuples_preserving_primitive_types(
            m,
            t,
            PrimitiveType::Triangle,
            PrimitiveType::Triangle);

        REQUIRE(fci.size() == 1);
        for (const Tuple& f : fci) {
            CHECK(m.id(f, PrimitiveType::Vertex) == 0);
            CHECK(m.id(f, PrimitiveType::Tetrahedron) == 0);
        }
    }
}