#include <stdlib.h>
#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("1D_initialize", "[mesh_creation],[tuple_1d]")
{
    SECTION("init from RowVectors2l")
    {
        DEBUG_EdgeMesh m;
        RowVectors2l lines;
        lines.resize(3, 2);
        lines << 0, 1, 1, 2, 2, 3;

        m.initialize(lines);

        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 3);

        REQUIRE(m.is_connectivity_valid());

        const Tuple t0 = edges[0];
        const Tuple t1 = edges[1];
        const Tuple t2 = edges[2];
        REQUIRE(m.is_valid_slow(t0));
        REQUIRE(m.is_valid_slow(t1));
        REQUIRE(m.is_valid_slow(t2));
    }
    SECTION("init single line")
    {
        DEBUG_EdgeMesh m = single_line();

        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 2);
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);

        REQUIRE(m.is_connectivity_valid());

        const Tuple t = edges[0];
        REQUIRE(m.is_valid_slow(t));
    }
    SECTION("init multiple lines")
    {
        DEBUG_EdgeMesh m = multiple_lines();

        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);

        REQUIRE(m.is_connectivity_valid());

        const Tuple t0 = edges[0];
        const Tuple t1 = edges[1];
        const Tuple t2 = edges[2];
        const Tuple t3 = edges[3];
        const Tuple t4 = edges[4];
        REQUIRE(m.is_valid_slow(t0));
        REQUIRE(m.is_valid_slow(t1));
        REQUIRE(m.is_valid_slow(t2));
        REQUIRE(m.is_valid_slow(t3));
        REQUIRE(m.is_valid_slow(t4));
    }
    SECTION("init loop lines")
    {
        DEBUG_EdgeMesh m = loop_lines();

        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 6);

        REQUIRE(m.is_connectivity_valid());

        const Tuple t0 = edges[0];
        const Tuple t1 = edges[1];
        const Tuple t2 = edges[2];
        const Tuple t3 = edges[3];
        const Tuple t4 = edges[4];
        const Tuple t5 = edges[5];
        REQUIRE(m.is_valid_slow(t0));
        REQUIRE(m.is_valid_slow(t1));
        REQUIRE(m.is_valid_slow(t2));
        REQUIRE(m.is_valid_slow(t3));
        REQUIRE(m.is_valid_slow(t4));
        REQUIRE(m.is_valid_slow(t5));
    }
    SECTION("init self loop")
    {
        DEBUG_EdgeMesh m = self_loop();

        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 1);
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);

        REQUIRE(m.is_connectivity_valid());

        const Tuple t0 = edges[0];
        REQUIRE(m.is_valid_slow(t0));
    }
}

TEST_CASE("1D_single_line", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = single_line();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 2);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Edge) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Edge) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);
    }
}

TEST_CASE("1D_single_line", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = single_line();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 2);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Edge) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Edge) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);
    }
}