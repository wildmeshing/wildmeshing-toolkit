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
    DEBUG_EdgeMesh m;
    std::vector<Tuple> edges, vertices;

    SECTION("init from RowVectors2l")
    {
        RowVectors2l lines;
        lines.resize(3, 2);
        lines << 0, 1, 1, 2, 2, 3;

        m.initialize(lines);

        vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 3);

        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("init single line")
    {
        m = single_line();

        vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 2);
        edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);

        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("init multiple lines")
    {
        m = multiple_lines();

        vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);

        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("init loop lines")
    {
        m = loop_lines();

        vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 6);

        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("init self loop")
    {
        m = self_loop();

        vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 1);
        edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);

        REQUIRE(m.is_connectivity_valid());
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        REQUIRE(m.is_valid(edges[i] ));
    }
}

TEST_CASE("1D_single_line", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = single_line();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 2);
        CHECK(m.id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m.id(vertices[0], PrimitiveType::Edge) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Edge) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);
    }
}

TEST_CASE("1D_multiple_lines", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = multiple_lines();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        for (int64_t i = 0; i < 6; ++i) {
            CHECK(m.id(vertices[i], PrimitiveType::Vertex) == i);
        }
        CHECK(m.id(vertices[0], PrimitiveType::Edge) == 0);
        CHECK(m.id(vertices[5], PrimitiveType::Edge) == 4);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
        for (int64_t i = 0; i < 5; ++i) {
            CHECK(m.id(edges[i], PrimitiveType::Edge) == i);
        }
    }
}

TEST_CASE("1D_loop_lines", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = loop_lines();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        for (int64_t i = 0; i < 6; ++i) {
            CHECK(m.id(vertices[i], PrimitiveType::Vertex) == i);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 6);
        for (int64_t i = 0; i < 6; ++i) {
            CHECK(m.id(edges[i], PrimitiveType::Edge) == i);
        }
    }
}


TEST_CASE("1D_self_loop", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = self_loop();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 1);
        CHECK(m.id(vertices[0], PrimitiveType::Vertex) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);
    }
}

TEST_CASE("1D_random_switches", "[tuple_operation],[tuple_1d]")
{
    DEBUG_EdgeMesh m = loop_lines();
    SECTION("vertices")
    {
        const std::vector<Tuple> vertex_tuples = m.get_all(PrimitiveType::Vertex);
        for (size_t i = 0; i < vertex_tuples.size(); ++i) {
            Tuple t = vertex_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 2) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1:
                    if (!m.is_boundary_vertex(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Edge);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid(t));
            }
        }
    }

    SECTION("edges")
    {
        const std::vector<Tuple> edge_tuples = m.get_all(PrimitiveType::Edge);
        for (size_t i = 0; i < edge_tuples.size(); ++i) {
            Tuple t = edge_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 2) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1:
                    if (!m.is_boundary_vertex(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Edge);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid(t));
            }
        }
    }
}

TEST_CASE("1D_is_boundary", "[tuple_1d]")
{
    DEBUG_EdgeMesh m;
    size_t n_boundary_vertices_expected = std::numeric_limits<size_t>::max();

    SECTION("single_line")
    {
        m = single_line();
        n_boundary_vertices_expected = 2;
    }

    SECTION("multiple_lines")
    {
        m = multiple_lines();
        n_boundary_vertices_expected = 2;
    }

    SECTION("loop_lines")
    {
        m = loop_lines();
        n_boundary_vertices_expected = 0;
    }

    SECTION("two_line_loop")
    {
        m = two_line_loop();
        n_boundary_vertices_expected = 0;
    }

    SECTION("self_loop")
    {
        m = self_loop();
        n_boundary_vertices_expected = 0;
    }

    // count boundary vertices
    size_t n_boundary_vertices = 0;
    for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
        if (m.is_boundary_vertex(v)) {
            ++n_boundary_vertices;
        }
    }

    CHECK(n_boundary_vertices == n_boundary_vertices_expected);
}

TEST_CASE("1D_double_switches", "[tuple_operation],[tuple_1d]")
{
    // checking for every tuple t:
    // (1) t.switch_vertex().switch_vertex() == t
    // (2) t.switch_edge().switch_edge() == t

    DEBUG_EdgeMesh m;
    SECTION("single_line")
    {
        m = single_line();
    }
    SECTION("multiple_lines")
    {
        m = multiple_lines();
    }
    SECTION("two_line_loop")
    {
        m = two_line_loop();
    }
    SECTION("loop_lines")
    {
        m = loop_lines();
    }
    SECTION("self_loop")
    {
        m = self_loop();
    }

    // vertices
    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    for (const auto& t : vertices) {
        const Tuple t_after_v = m.switch_vertex(m.switch_vertex(t));
        CHECK(t == t_after_v);
        if (!m.is_boundary_vertex(t)) {
            const Tuple t_after_e = m.switch_edge(m.switch_edge(t));
            CHECK(t == t_after_e);
        }
    }

    // edges
    const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
    for (const auto& t : edges) {
        const Tuple t_after_v = m.switch_vertex(m.switch_vertex(t));
        CHECK(t == t_after_v);
        if (!m.is_boundary_vertex(t)) {
            const Tuple t_after_e = m.switch_edge(m.switch_edge(t));
            CHECK(t == t_after_e);
        }
    }
}
