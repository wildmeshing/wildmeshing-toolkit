#include <catch2/catch_test_macros.hpp>

#include <queue>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>
#include <wmtk/utils/DynamicArray.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace utils;

TEST_CASE("dynamic_array", "[DynamicArray]")
{
    DynamicArray<int64_t, 3> arr;
    constexpr uint64_t ArraySize = decltype(arr)::array_size();
    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == 0);
    CHECK(arr.capacity() == ArraySize);

    for (uint64_t i = 0; i < ArraySize; ++i) {
        arr.emplace_back(i);
    }

    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == ArraySize);
    CHECK(arr.capacity() == ArraySize);

    for (uint64_t i = ArraySize; i < 10; ++i) {
        arr.emplace_back(i);
    }
    CHECK(arr.uses_vector());
    CHECK(arr.size() == 10);
    CHECK(arr.capacity() >= 10);

    for (uint64_t i = 0; i < arr.size(); ++i) {
        CHECK(arr[i] == i);
        arr[i] = 0;
        CHECK(arr[i] == 0);
        arr[i] = i;
    }

    {
        uint64_t i = 0;
        for (const int v : arr) {
            CHECK(v == i++);
        }
    }
}

TEST_CASE("visited_array", "[DynamicArray]")
{
    simplex::internal::VisitedArray<int64_t> visited;
    const auto& internal_array = visited.visited_array();

    CHECK_FALSE(visited.is_visited(42));
    CHECK(visited.is_visited(42));
    CHECK(internal_array.size() == 1);
}

TEST_CASE("visited_array_bfs", "[DynamicArray][VisitedArray]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    simplex::internal::VisitedArray<int64_t> visited_cells;

    int64_t vertex_id;
    int64_t expected_size;
    SECTION("interior")
    {
        vertex_id = 4;
        expected_size = 6;
    }
    SECTION("boundary_1")
    {
        vertex_id = 1;
        expected_size = 3;
    }
    SECTION("boundary_2")
    {
        vertex_id = 6;
        expected_size = 1;
    }

    simplex::Simplex v(m, PrimitiveType::Vertex, m.vertex_tuple_from_id(vertex_id));

    std::queue<Tuple> q;
    {
        Tuple t = v.tuple();
        visited_cells.is_visited(t.global_cid());
        if (!m.is_boundary_edge(t)) {
            q.push(m.switch_tuple(t, PrimitiveType::Triangle));
        }
        t = m.switch_edge(t);
        if (!m.is_boundary_edge(t)) {
            q.push(m.switch_tuple(t, PrimitiveType::Triangle));
        }
    }

    while (!q.empty()) {
        Tuple t = q.front();
        q.pop();

        std::array<Tuple, 2> edges = {t, m.switch_edge(t)};
        for (Tuple e : edges) {
            if (!m.is_boundary_edge(e)) {
                e = m.switch_face(e);
                if (!visited_cells.is_visited(e.global_cid())) {
                    q.push(e);
                }
            }
        }
    }

    CHECK(visited_cells.visited_array().size() == expected_size);
}

TEST_CASE("visited_array_bfs_lower_dimension", "[DynamicArray][VisitedArray]")
{
    tests::DEBUG_TriMesh m = tests::hex_plus_two();

    simplex::internal::VisitedArray<int64_t> visited_cells;
    simplex::internal::VisitedArray<simplex::RawSimplex> visited_edges;

    int64_t vertex_id = -1;
    int64_t expected_cells_size = -1;
    int64_t expected_edges_size = -1;
    SECTION("interior")
    {
        vertex_id = 4;
        expected_cells_size = 6;
        expected_edges_size = 6;
    }
    SECTION("boundary_1")
    {
        vertex_id = 1;
        expected_cells_size = 3;
        expected_edges_size = 4;
    }
    SECTION("boundary_2")
    {
        vertex_id = 6;
        expected_cells_size = 1;
        expected_edges_size = 2;
    }

    simplex::Simplex v(m, PrimitiveType::Vertex, m.vertex_tuple_from_id(vertex_id));

    std::queue<Tuple> q;

    visited_cells.is_visited(v.tuple().global_cid());
    q.push(v.tuple());

    while (!q.empty()) {
        Tuple t = q.front();
        q.pop();

        std::array<Tuple, 2> edges = {t, m.switch_edge(t)};
        for (Tuple e : edges) {
            simplex::RawSimplex e_simplex(m, simplex::Simplex(m, PrimitiveType::Edge, e));
            visited_edges.is_visited(e_simplex);

            if (!m.is_boundary_edge(e)) {
                e = m.switch_face(e);
                if (!visited_cells.is_visited(e.global_cid())) {
                    q.push(e);
                }
            }
        }
    }

    CHECK(visited_cells.visited_array().size() == expected_cells_size);
    CHECK(visited_edges.visited_array().size() == expected_edges_size);
}