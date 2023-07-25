
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
    long genus = -1;
    long simply_connected_components = -1;
    bool is_oriented = false;
    std::array<long, 3> simplex_counts = std::array<long,3>{{-1,-1,-1}};
};
void run_debug_trimesh(const DEBUG_TriMesh& m, const MeshDebugInfo& info)
{
    // validate that the info is ok
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

    auto v_tups = m.get_all(PrimitiveType::Vertex);
    auto e_tups = m.get_all(PrimitiveType::Edge);
    auto f_tups = m.get_all(PrimitiveType::Face);
    for (const auto& v_tup : v_tups) {
        fmt::print("vertex {} is active", m.id( v_tup, PrimitiveType::Vertex));
    }
    for (const auto& e_tup : e_tups) {
        long id = m.id(e_tup, PrimitiveType::Edge);
        long a = m.id(e_tup, PrimitiveType::Vertex);
        long b = m.id(m.switch_tuple(e_tup, PrimitiveType::Vertex), PrimitiveType::Vertex );
        fmt::print("edge {} is active with vertices {} {}", id, a, b);
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
        fmt::print("face {} is active with vertices {} {} {}", id, a, b, c);
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
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{3, 3, 3}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_one_ear")
{
    DEBUG_TriMesh m;
    m = one_ear();
    MeshDebugInfo info;
    info.name = "one_ear";
    info.genus = 0;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{3, 3, 3}};
    run_debug_trimesh(m, info);
}
