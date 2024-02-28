#include <stdlib.h>
#include <wmtk/utils/trimesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("2D_initialize", "[mesh_creation],[tuple_2d]")
{
    DEBUG_TriMesh m;
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};

    SECTION("init with FV, FE, FF, VF, EF")
    {
        m = single_triangle();
    }
    SECTION("init directly from RowVectors3l")
    {
        m.initialize(tris);
    }

    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 3);
    const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
    REQUIRE(edges.size() == 3);
    const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
    REQUIRE(faces.size() == 1);


    REQUIRE(m.is_connectivity_valid());

    const Tuple t = m.get_all(PrimitiveType::Triangle)[0];
    REQUIRE(m.is_valid_slow(t));
}

TEST_CASE("2D_1_triangle", "[tuple_generation],[tuple_2d]")
{
    DEBUG_TriMesh m = single_triangle();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 3);
        CHECK(m.id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m.id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m.id(vertices[0], PrimitiveType::Triangle) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Triangle) == 0);
        CHECK(m.id(vertices[2], PrimitiveType::Triangle) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 3);
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
        REQUIRE(faces.size() == 1);
        CHECK(m.id(faces[0], PrimitiveType::Triangle) == 0);
    }
}

TEST_CASE("2D_2_triangles", "[tuple_generation],[tuple_2d]")
{
    // 	   v3     /
    //     / \    /
    // 	  /f1 \   /
    // v2 -----v1 /
    // 	  \f0 /   /
    //     \ /    /
    // 	    v0    /

    DEBUG_TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(2, 3);
        tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<int64_t, 3, 1>{2, 1, 3};
        m.initialize(tris);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        CHECK(m.id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m.id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m.id(vertices[3], PrimitiveType::Vertex) == 3);
        CHECK(m.id(vertices[0], PrimitiveType::Triangle) == 0);
        CHECK(m.id(vertices[3], PrimitiveType::Triangle) == 1);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 5);
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
        REQUIRE(faces.size() == 2);
        CHECK(m.id(faces[0], PrimitiveType::Triangle) == 0);
        CHECK(m.id(faces[1], PrimitiveType::Triangle) == 1);
    }
}

// for every quiry do a require
TEST_CASE("2D_random_switches", "[tuple_operation],[tuple_2d]")
{
    DEBUG_TriMesh m = interior_edge();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertex_tuples = m.get_all(PrimitiveType::Vertex);
        for (size_t i = 0; i < vertex_tuples.size(); ++i) {
            Tuple t = vertex_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary_edge(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Triangle);
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
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary_edge(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Triangle);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid_slow(t));
            }
        }
    }

    SECTION("faces")
    {
        const std::vector<Tuple> face_tuples = m.get_all(PrimitiveType::Triangle);
        for (size_t i = 0; i < face_tuples.size(); ++i) {
            Tuple t = face_tuples[i];
            for (size_t j = 0; j < 10; j++) {
                switch (rand() % 3) {
                case 0: t = m.switch_tuple(t, PrimitiveType::Vertex); break;
                case 1: t = m.switch_tuple(t, PrimitiveType::Edge); break;
                case 2:
                    if (!m.is_boundary_edge(t)) {
                        t = m.switch_tuple(t, PrimitiveType::Triangle);
                    }
                    break;
                default: break;
                }
                CHECK(m.is_valid_slow(t));
            }
        }
    }
}

bool tuple_equal(const DEBUG_TriMesh& m, const Tuple& t0, const Tuple& t1)
{
    const int64_t v0 = m.id(t0, PrimitiveType::Vertex);
    const int64_t e0 = m.id(t0, PrimitiveType::Edge);
    const int64_t f0 = m.id(t0, PrimitiveType::Triangle);
    const int64_t v1 = m.id(t1, PrimitiveType::Vertex);
    const int64_t e1 = m.id(t1, PrimitiveType::Edge);
    const int64_t f1 = m.id(t1, PrimitiveType::Triangle);
    return (v0 == v1) && (e0 == e1) && (f0 == f1);
}


TEST_CASE("2D_double_switches", "[tuple_operation],[tuple_2d]")
{
    // checking for every tuple t:
    // (1) t.switch_vertex().switch_vertex() == t
    // (2) t.switch_edge().switch_edge() == t
    // (3) t.switch_tri().switch_tri() == t

    DEBUG_TriMesh m = interior_edge();
    auto try_switch = [&](const Tuple& t) {
        const Tuple t_after_v = m.switch_tuples(t, {PrimitiveType::Vertex, PrimitiveType::Vertex});
        CHECK(tuple_equal(m, t, t_after_v));
        const Tuple t_after_e = m.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Edge});
        CHECK(tuple_equal(m, t, t_after_e));
        if (!m.is_boundary_edge(t)) {
            const Tuple t_after_f =
                m.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Triangle});
            CHECK(tuple_equal(m, t, t_after_f));
        }
    };

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 5);
        for (const auto& t : vertices) {
            try_switch(t);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 7);
        for (const auto& t : edges) {
            try_switch(t);
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
        REQUIRE(faces.size() == 3);
        for (const auto& t : faces) {
            try_switch(t);
        }
    }
}

TEST_CASE("2D_switch_sequences", "[tuple_operation],[tuple_2d]")
{
    // checking for every tuple t:
    // (1) t.switch_vertex().switch_edge() == t.switch_tuples(t,{V,E});
    // (2) t.switch_edge().switch_vertex() == t.switch_tuples(t,{E,V});
    // (3) t.switch_tri().switch_edge() == t = t.switch_tuples(t,{T,E})

    DEBUG_TriMesh m = interior_edge();

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 5);
        for (const auto& t : vertices) {
            const Tuple t_long = m.switch_edge(m.switch_vertex(t));
            const Tuple t_short = m.switch_tuples(t, {PrimitiveType::Vertex, PrimitiveType::Edge});
            const Tuple t_short_u =
                m.switch_tuples_unsafe(t, {PrimitiveType::Vertex, PrimitiveType::Edge});
            CHECK(tuple_equal(m, t_long, t_short));
            CHECK(tuple_equal(m, t_long, t_short_u));
            if (!m.is_boundary_edge(t)) {
                const Tuple _t_long = m.switch_edge(m.switch_face(t));
                const Tuple _t_short =
                    m.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
                const Tuple _t_short_u =
                    m.switch_tuples_unsafe(t, {PrimitiveType::Triangle, PrimitiveType::Edge});

                CHECK(tuple_equal(m, _t_long, _t_short));
                CHECK(tuple_equal(m, _t_long, _t_short_u));
            }
        }
    }
}

TEST_CASE("2D_next_next_next", "[tuple_operation],[tuple_2d]")
{
    DEBUG_TriMesh m = interior_edge();
    auto loop_all = [&](Tuple t) {
        for (int j = 0; j < 3; ++j) {
            t = m.switch_tuples(t, {PrimitiveType::Vertex, PrimitiveType::Edge});
        }
        return t;
    };

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 5);
        for (const Tuple& t : vertices) {
            const Tuple t_iter = loop_all(t);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 7);
        for (const Tuple& t : edges) {
            const Tuple t_iter = loop_all(t);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
        REQUIRE(faces.size() == 3);
        for (const Tuple& t : faces) {
            const Tuple t_iter = loop_all(t);
            CHECK(tuple_equal(m, t, t_iter));
        }
    }
}

TEST_CASE("2D_one_ring_iteration", "[tuple_operation],[tuple_2d]")
{
    DEBUG_TriMesh m;
    {
        RowVectors3l tris;
        tris.resize(6, 3);
        tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<int64_t, 3, 1>{0, 2, 3};
        tris.row(2) = Eigen::Matrix<int64_t, 3, 1>{0, 3, 4};
        tris.row(3) = Eigen::Matrix<int64_t, 3, 1>{0, 4, 5};
        tris.row(4) = Eigen::Matrix<int64_t, 3, 1>{0, 5, 6};
        tris.row(5) = Eigen::Matrix<int64_t, 3, 1>{0, 6, 1};
        m.initialize(tris);
    }

    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 7);
    for (const auto& t : vertices) {
        // face-edge switch
        Tuple t_iter = t;
        for (size_t i = 0; i < 6; ++i) {
            if (m.is_boundary_edge(t_iter)) {
                break;
            }
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Triangle);
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            if (tuple_equal(m, t, t_iter)) {
                break;
            }
        }
        CHECK((tuple_equal(m, t, t_iter) || m.is_boundary_edge(t_iter)));
        // edge-face switch
        t_iter = t;
        for (size_t i = 0; i < 6; ++i) {
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Edge);
            if (m.is_boundary_edge(t_iter)) {
                break;
            }
            t_iter = m.switch_tuple(t_iter, PrimitiveType::Triangle);
            if (tuple_equal(m, t, t_iter)) {
                break;
            }
        }
        CHECK((tuple_equal(m, t, t_iter) || m.is_boundary_edge(t_iter)));
    }
}

TEST_CASE("2D_is_boundary", "[tuple_2d]")
{
    DEBUG_TriMesh m = edge_region();

    size_t n_boundary_edges = 0;
    for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
        if (m.is_boundary_edge(e)) {
            ++n_boundary_edges;
        }
    }
    CHECK(n_boundary_edges == 8);

    size_t n_boundary_vertices = 0;
    for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
        if (m.is_boundary_vertex(v)) {
            ++n_boundary_vertices;
        }
    }
    CHECK(n_boundary_vertices == 8);


    const Tuple t1 = m.edge_tuple_between_v1_v2(0, 1, 1);
    CHECK(m.is_boundary_edge(t1));
    CHECK(m.is_boundary_vertex(t1));

    const Tuple t2 = m.switch_edge(t1);
    CHECK(!m.is_boundary_edge(t2));
    CHECK(m.is_boundary_vertex(t2));

    const Tuple t3 = m.switch_tuple(t2, PrimitiveType::Vertex);
    CHECK(!m.is_boundary_edge(t3));
    CHECK(!m.is_boundary_vertex(t3));
}
