
#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include <array>

using namespace wmtk;
using namespace wmtk::tests;
namespace {
struct MeshDebugInfo
{
    std::string name = "RENAME ME";
    long boundary_curves = -1;// number of distinct boundary curves in the mesh
    long genus = -1;// TODO (also what definition of genus?)
    long simply_connected_components = -1;// TODO (face-face connected topologies)
    bool is_oriented = false;// TODO (make sure nface neighbors use opposite orientation
    // the number of simplices (vertex, edge, then face)
    std::array<long, 3> simplex_counts = std::array<long,3>{{-1,-1,-1}};
};
void run_debug_trimesh(const DEBUG_TriMesh& m, const MeshDebugInfo& info)
{
    fmt::print("Running run_debug_trimesh  on {}\n", info.name);

    // validate that the info is ok
    REQUIRE(info.boundary_curves >= 0);
    REQUIRE(info.genus >= 0);
    REQUIRE(info.simply_connected_components>= 0);
    for(const long count: info.simplex_counts) {
        REQUIRE(count >= 0);
    }

    REQUIRE(m.is_connectivity_valid());

    // TODO :in the future we should check for some topological info
    // CHECK(genus(m) == info.genus);
    // CHECK(simply_connected_components(m) == info.simply_connected_components);
    // for(long j = 0; j < 3; ++j) {
    //    CHECK(simplex_count(m,j) == info.simplex_counts);
    //}

    auto [v_count, e_count, f_count] = info.simplex_counts;
    auto v_tups = m.get_all(PrimitiveType::Vertex);
    auto e_tups = m.get_all(PrimitiveType::Edge);
    auto f_tups = m.get_all(PrimitiveType::Face);

    REQUIRE(v_count == v_tups.size());
    REQUIRE(e_count == e_tups.size());
    REQUIRE(f_count == f_tups.size());
    for (const auto& v_tup : v_tups) {
        fmt::print("vertex {} is active\n", m.id( v_tup, PrimitiveType::Vertex));
    }
    for (const auto& e_tup : e_tups) {
        long id = m.id(e_tup, PrimitiveType::Edge);
        long a = m.id(e_tup, PrimitiveType::Vertex);
        long b = m.id(m.switch_tuple(e_tup, PrimitiveType::Vertex), PrimitiveType::Vertex );
        fmt::print("edge {} is active with vertices {} {}\n", id, a, b);
    }
    for (const auto& f_tup : f_tups) {
        long id = m.id(f_tup, PrimitiveType::Face);
        long a = m.id(f_tup, PrimitiveType::Vertex);
        long b = m.id(m.switch_tuple(f_tup, PrimitiveType::Vertex), PrimitiveType::Vertex );
        long c = m.id(
            m.switch_tuple(
                m.switch_tuple(f_tup, PrimitiveType::Vertex
                    ),
                PrimitiveType::Edge
                ),
            PrimitiveType::Vertex
            );
        fmt::print("face {} is active with vertices {} {} {}\n", id, a, b, c);
    }
}
} // namespace

TEST_CASE("test_debug_trimeshes_single_triangle")
{
    DEBUG_TriMesh m;
    m = single_triangle();
    MeshDebugInfo info;
    info.name = "single_triangle";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{3, 3, 1}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_quad")
{
    DEBUG_TriMesh m;
    m = quad();
    MeshDebugInfo info;
    info.name = "quad";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{4, 5, 2}};
    run_debug_trimesh(m, info);
}
TEST_CASE("test_debug_trimeshes_two_neighbors")
{
    DEBUG_TriMesh m;
    m = two_neighbors();
    MeshDebugInfo info;
    info.name = "two_neighbors";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{5, 7, 3}};
    run_debug_trimesh(m, info);
}
TEST_CASE("test_debug_trimeshes_three_neighbors")
{
    DEBUG_TriMesh m;
    m = three_neighbors();
    MeshDebugInfo info;
    info.name = "three_neighbors";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{6, 9, 4}};
    run_debug_trimesh(m, info);
}
TEST_CASE("test_debug_trimeshes_three_individuals")
{
    DEBUG_TriMesh m;
    m = three_individuals();
    MeshDebugInfo info;
    info.name = "three_individuals";
    info.genus = 0;
    info.boundary_curves = 3;// each triangle is individual
    info.simply_connected_components = 3;
    info.simplex_counts = std::array<long, 3>{{6, 9, 3}};
    run_debug_trimesh(m, info);
}
TEST_CASE("test_debug_trimeshes_tetrahedron")
{
    DEBUG_TriMesh m;
    m = tetrahedron();
    MeshDebugInfo info;
    info.name = "tetrahedron";
    info.genus = 0;
    info.boundary_curves = 0;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{4, 6, 4}};
    run_debug_trimesh(m, info);
}
TEST_CASE("test_debug_trimeshes_hex_plus_two")
{
    DEBUG_TriMesh m;
    m = hex_plus_two();
    MeshDebugInfo info;
    info.name = "hex_plus_two";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{9, 16, 8}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_edge_region")
{
    DEBUG_TriMesh m;
    m = edge_region();
    MeshDebugInfo info;
    info.name = "edge_region";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{10, 19, 10}};
    run_debug_trimesh(m, info);
}
