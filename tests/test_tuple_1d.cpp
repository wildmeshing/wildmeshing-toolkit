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

TEST_CASE("1D_multiple_lines", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = multiple_lines();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m._debug_id(vertices[3], PrimitiveType::Vertex) == 3);
        CHECK(m._debug_id(vertices[4], PrimitiveType::Vertex) == 4);
        CHECK(m._debug_id(vertices[5], PrimitiveType::Vertex) == 5);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Edge) == 0);
        CHECK(m._debug_id(vertices[5], PrimitiveType::Edge) == 4);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
    }
}

TEST_CASE("1D_loop_lines", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = loop_lines();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 6);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m._debug_id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m._debug_id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m._debug_id(vertices[3], PrimitiveType::Vertex) == 3);
        CHECK(m._debug_id(vertices[4], PrimitiveType::Vertex) == 4);
        CHECK(m._debug_id(vertices[5], PrimitiveType::Vertex) == 5);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 6);
    }
}


TEST_CASE("1D_self_loop", "[tuple_generation], [tuple_1d]")
{
    DEBUG_EdgeMesh m = self_loop();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 1);
        CHECK(m._debug_id(vertices[0], PrimitiveType::Vertex) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 1);
    }
}

TEST_CASE("1D_random_switches", "[tuple_operation],[tuple_1d]")
{
    // DEBUG_EdgeMesh m = single_line();
    // DEBUG_EdgeMesh m = multiple_lines();
    DEBUG_EdgeMesh m = loop_lines();
    // DEBUG_EdgeMesh m = two_line_loop();
    // DEBUG_EdgeMesh m = self_loop();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertex_tuples = m.get_all(PrimitiveType::Vertex);
        for (size_t i = 0; i < vertex_tuples.size(); ++i) {
            Tuple t = vertex_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 2) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1:
                    if (!m.is_boundary(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Edge);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid_slow(t));
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
                    if (!m.is_boundary(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Edge);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid_slow(t));
            }
        }
    }
}

TEST_CASE("1D_is_boundary", "[tuple_1d]")
{
    SECTION("single_line")
    {
        DEBUG_EdgeMesh m = single_line();

        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary(v)) {
                ++n_boundary_vertices;
            }
        }

        CHECK(n_boundary_vertices == 2);
    }

    SECTION("multiple_lines")
    {
        DEBUG_EdgeMesh m = multiple_lines();

        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary(v)) {
                ++n_boundary_vertices;
            }
        }

        CHECK(n_boundary_vertices == 2);
    }

    SECTION("loop_lines")
    {
        DEBUG_EdgeMesh m = loop_lines();

        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary(v)) {
                ++n_boundary_vertices;
            }
        }

        CHECK(n_boundary_vertices == 0);
    }

    SECTION("two_line_loop")
    {
        DEBUG_EdgeMesh m = two_line_loop();

        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary(v)) {
                ++n_boundary_vertices;
            }
        }

        CHECK(n_boundary_vertices == 0);
    }

    SECTION("self_loop")
    {
        DEBUG_EdgeMesh m = self_loop();

        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary(v)) {
                ++n_boundary_vertices;
            }
        }

        CHECK(n_boundary_vertices == 0);
    }
}

bool tuple_equal(const EdgeMesh& m, const Tuple& t0, const Tuple& t1)
{
    const auto l = wmtk::logger().level();
    wmtk::logger().set_level(spdlog::level::err);
    const long v0 = m._debug_id(t0, PrimitiveType::Vertex);
    const long e0 = m._debug_id(t0, PrimitiveType::Edge);
    const long v1 = m._debug_id(t1, PrimitiveType::Vertex);
    const long e1 = m._debug_id(t1, PrimitiveType::Edge);
    wmtk::logger().set_level(l);
    return (v0 == v1) && (e0 == e1);
}

TEST_CASE("1D_double_switches", "[tuple_operation],[tuple_1d]")
{
    // checking for every tuple t:
    // (1) t.switch_vertex().switch_vertex() == t
    // (2) t.switch_edge().switch_edge() == t

    // DEBUG_EdgeMesh m = single_line();
    DEBUG_EdgeMesh m = multiple_lines();
    // DEBUG_EdgeMesh m = loop_lines();
    // DEBUG_EdgeMesh m = two_line_loop();
    // DEBUG_EdgeMesh m = self_loop();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        for (const auto& t : vertices) {
            const Tuple t_after_v = m.switch_vertex(m.switch_vertex(t));
            CHECK(tuple_equal(m, t, t_after_v));
            if (!m.is_boundary(t)) {
                const Tuple t_after_e = m.switch_edge(m.switch_edge(t));
                CHECK(tuple_equal(m, t, t_after_e));
            }
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        for (const auto& t : edges) {
            const Tuple t_after_v = m.switch_vertex(m.switch_vertex(t));
            CHECK(tuple_equal(m, t, t_after_v));
            if (!m.is_boundary(t)) {
                const Tuple t_after_e = m.switch_edge(m.switch_edge(t));
                CHECK(tuple_equal(m, t, t_after_e));
            }
        }
    }
}