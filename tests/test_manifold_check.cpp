#include <catch2/catch_test_macros.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/ManifoldCheck.hpp>

#include <array>
#include <set>
#include <vector>

using namespace wmtk;
using namespace wmtk::utils;

// The shared manifold checks on the base meshes (connectivity only): K is a set of cell ids.
namespace {

CellTest cells_in(const std::set<size_t>& K)
{
    return [K](const size_t id) { return K.count(id) > 0; };
}

std::vector<size_t> all_cells(const size_t n)
{
    std::vector<size_t> ids(n);
    for (size_t i = 0; i < n; ++i) {
        ids[i] = i;
    }
    return ids;
}

/// An octahedron around vertex 0: vertex 1 at +x, 2 at -x, 3 at +y, 4 at -y, 5 at +z, 6 at -z;
/// tet i (i = 0..7) fills the octant whose signs are the bits of i (bit 0 x, bit 1 y, bit 2 z,
/// set = negative), oriented positively. Vertex 0 is interior, the others on the boundary.
void init_octahedron(TetMesh& m)
{
    std::vector<std::array<size_t, 4>> tets;
    for (size_t i = 0; i < 8; ++i) {
        const bool nx = i & 1, ny = i & 2, nz = i & 4;
        const size_t x = nx ? 2 : 1, y = ny ? 4 : 3, z = nz ? 6 : 5;
        // det(x, y, z) has the sign of (+-1)(+-1)(+-1): swap two if negative
        if (nx ^ ny ^ nz) {
            tets.push_back({{0, x, z, y}});
        } else {
            tets.push_back({{0, x, y, z}});
        }
    }
    m.init(7, tets);
}

size_t octant(const bool nx, const bool ny, const bool nz)
{
    return size_t(nx) | (size_t(ny) << 1) | (size_t(nz) << 2);
}

} // namespace

TEST_CASE("manifold_check_tri_fan", "[manifold_check][2d]")
{
    // a hexagon fanned around its centre 0 (interior), corners 1..6
    TriMesh m;
    m.init(7, {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}, {{0, 4, 5}}, {{0, 5, 6}}, {{0, 6, 1}}});
    const auto centre = m.tuple_from_vertex(0);

    CHECK(vertex_is_manifold(m, centre, cells_in({})));
    CHECK(vertex_is_manifold(m, centre, cells_in({0, 1, 2, 3, 4, 5})));
    CHECK(vertex_is_manifold(m, centre, cells_in({0, 1})));
    CHECK(vertex_is_manifold(m, centre, cells_in({0, 1, 2, 3, 4})));
    // the fan wraps around: faces 5 and 0 share edge (0, 1)
    CHECK(vertex_is_manifold(m, centre, cells_in({5, 0})));
    // two fans meeting at the centre only
    CHECK_FALSE(vertex_is_manifold(m, centre, cells_in({0, 3})));
    CHECK_FALSE(vertex_is_manifold(m, centre, cells_in({0, 1, 3, 4})));

    CHECK(nonmanifold_vertices(m, all_cells(6), cells_in({0, 3})) == std::vector<size_t>{0});
    CHECK(nonmanifold_vertices(m, all_cells(6), cells_in({0, 1, 2})).empty());
    // a cell outside K gives K's non-manifold vertices among its own
    CHECK(nonmanifold_vertices(m, {1}, cells_in({0, 3})) == std::vector<size_t>{0});
}

TEST_CASE("manifold_check_tri_boundary", "[manifold_check][2d]")
{
    // three faces around vertex 0 on the mesh's boundary: an open fan
    TriMesh m;
    m.init(5, {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}});
    const auto v = m.tuple_from_vertex(0);

    CHECK(vertex_is_manifold(m, v, cells_in({0, 1, 2})));
    CHECK(vertex_is_manifold(m, v, cells_in({0, 1})));
    CHECK(vertex_is_manifold(m, v, cells_in({1})));
    // the two ends of the open fan do not close up around the vertex
    CHECK_FALSE(vertex_is_manifold(m, v, cells_in({0, 2})));
}

TEST_CASE("manifold_check_tet_interior_vertex", "[manifold_check][3d]")
{
    TetMesh m;
    init_octahedron(m);
    const auto centre = m.tuple_from_vertex(0);
    std::set<size_t> all;
    for (size_t i = 0; i < 8; ++i) {
        all.insert(i);
    }

    SECTION("balls and half balls")
    {
        CHECK(vertex_is_manifold(m, centre, cells_in({})));
        CHECK(vertex_is_manifold(m, centre, cells_in(all)));
        CHECK(vertex_is_manifold(m, centre, cells_in({octant(0, 0, 0)})));
        // the upper half: link a disk
        CHECK(vertex_is_manifold(
            m,
            centre,
            cells_in({octant(0, 0, 0), octant(1, 0, 0), octant(0, 1, 0), octant(1, 1, 0)})));
        // all but one octant: link a disk
        std::set<size_t> K = all;
        K.erase(octant(1, 1, 1));
        CHECK(vertex_is_manifold(m, centre, cells_in(K)));
        CHECK(nonmanifold_vertices(m, all_cells(8), cells_in(K)).empty());
    }

    SECTION("two octants meeting at the centre only")
    {
        const std::set<size_t> K = {octant(0, 0, 0), octant(1, 1, 1)};
        CHECK_FALSE(vertex_is_manifold(m, centre, cells_in(K)));
        CHECK(nonmanifold_vertices(m, all_cells(8), cells_in(K)) == std::vector<size_t>{0});
    }

    SECTION("two octants meeting along an edge")
    {
        // both above the xy plane, opposite in x and y: they share the edge from 0 to +z (5)
        const std::set<size_t> K = {octant(0, 0, 0), octant(1, 1, 0)};
        CHECK_FALSE(edge_is_manifold(m, m.tuple_from_edge({{0, 5}}), cells_in(K)));
        CHECK(edge_is_manifold(m, m.tuple_from_edge({{0, 1}}), cells_in(K)));
        // the vertex tests catch both ends too
        CHECK_FALSE(vertex_is_manifold(m, centre, cells_in(K)));
        CHECK_FALSE(vertex_is_manifold(m, m.tuple_from_vertex(5), cells_in(K)));
        CHECK(nonmanifold_vertices(m, all_cells(8), cells_in(K)) == std::vector<size_t>{0, 5});
    }

    SECTION("link an annulus: every edge manifold, the vertex not")
    {
        // all but two opposite octants, which share only the centre: K's tets around the centre
        // are joined through faces and every edge at it is manifold, but its link is the sphere
        // less two disjoint triangles
        std::set<size_t> K = all;
        K.erase(octant(0, 0, 0));
        K.erase(octant(1, 1, 1));
        for (size_t corner = 1; corner <= 6; ++corner) {
            CHECK(edge_is_manifold(m, m.tuple_from_edge({{0, corner}}), cells_in(K)));
            CHECK(vertex_is_manifold(m, m.tuple_from_vertex(corner), cells_in(K)));
        }
        CHECK_FALSE(vertex_is_manifold(m, centre, cells_in(K)));
        CHECK(nonmanifold_vertices(m, all_cells(8), cells_in(K)) == std::vector<size_t>{0});
    }
}

TEST_CASE("manifold_check_tet_boundary_vertex", "[manifold_check][3d]")
{
    // Seven tets around vertex 0 on the mesh's boundary. Its link is a triangle A B C (1 2 3)
    // around an inner triangle a b c (4 5 6), joined by six triangles; tet i = (0, link
    // triangle i). The faces (0, A, B), (0, B, C), (0, C, A) lie on the mesh's boundary; the
    // inner tet 6 = (0, a, b, c) has none at 0 (its face a b c, opposite 0, is on the boundary).
    TetMesh m;
    m.init(
        7,
        {{{0, 1, 2, 4}},
         {{0, 2, 5, 4}},
         {{0, 2, 3, 5}},
         {{0, 3, 6, 5}},
         {{0, 3, 1, 6}},
         {{0, 1, 4, 6}},
         {{0, 4, 5, 6}}});
    const auto v = m.tuple_from_vertex(0);

    CHECK(vertex_is_manifold(m, v, cells_in({0, 1, 2, 3, 4, 5, 6})));
    CHECK(vertex_is_manifold(m, v, cells_in({6})));
    CHECK(vertex_is_manifold(m, v, cells_in({0, 1, 2})));
    // the six outer tets: the link is an annulus, the inner tet a hole that does not reach the
    // space outside the mesh
    CHECK_FALSE(vertex_is_manifold(m, v, cells_in({0, 1, 2, 3, 4, 5})));
    // less one outer tet: the outer tet reaches the space outside the mesh, the inner one
    // reaches it only through its face a b c, away from the vertex, which does not count
    CHECK_FALSE(vertex_is_manifold(m, v, cells_in({1, 2, 3, 4, 5})));
}
