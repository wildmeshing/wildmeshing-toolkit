
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <queue>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
namespace {
struct MeshDebugInfo
{
    std::string name = "RENAME ME";
    long boundary_curves = -1; // number of distinct boundary curves in the mesh
    long genus = -1; // TODO (also what definition of genus?)
    long simply_connected_components = -1; // TODO (face-face connected topologies)
    bool is_oriented = false; // TODO (make sure nface neighbors use opposite orientation
    // the number of simplices (vertex, edge, then face)
    std::array<long, 3> simplex_counts = std::array<long, 3>{{-1, -1, -1}};
};

std::array<long, 3> trimesh_simplex_counts(const TriMesh& m)
{
    return std::array<long, 3>{
        {long(m.get_all(PrimitiveType::Vertex).size()),
         long(m.get_all(PrimitiveType::Edge).size()),
         long(m.get_all(PrimitiveType::Face).size())}};
}


/***
 * @brief compute the number of simply connected components in a mesh
 * @param m the mesh
 * @param component_ids output the component id of each face
 * @return the number of simply connected components
 */

int trimesh_simply_connected_components(const DEBUG_TriMesh& m, std::vector<int>& component_ids)
{
    component_ids.resize(m.capacity(PrimitiveType::Face), -1);
    const auto all_face_tuples = m.get_all(PrimitiveType::Face);
    int component_id = 0;
    // BFS
    for (auto face_tuple : all_face_tuples) {
        const long fid = m.id(face_tuple, PrimitiveType::Face);
        if (component_ids[fid] != -1) {
            continue;
        } // visited

        std::queue<long> q;
        q.push(fid);
        while (!q.empty()) {
            long cur_fid = q.front();
            q.pop();

            component_ids[cur_fid] = component_id;
            auto f_tuple = m.tuple_from_face_id(cur_fid);
            // push all adjacent faces
            for (int j = 0; j < 3; ++j) {
                if (!m.is_boundary_edge(f_tuple)) {
                    auto adj_face_tuple = m.switch_tuple(f_tuple, PrimitiveType::Face);
                    long adj_fid = m.id(adj_face_tuple, PrimitiveType::Face);
                    if (component_ids[adj_fid] == -1) {
                        q.push(adj_fid);
                    }
                }
                f_tuple = m.switch_edge(m.switch_vertex(f_tuple));
            }
        }

        component_id++;
    }

    return component_id;
}

int trimesh_simply_connected_components(const DEBUG_TriMesh& m)
{
    std::vector<int> component_ids;
    return trimesh_simply_connected_components(m, component_ids);
}

int trimesh_boundary_loops_count(const DEBUG_TriMesh& m)
{
    std::vector<int> boundary_loop_ids(m.capacity(PrimitiveType::Edge), -1);
    const auto all_edge_tuples = m.get_all(PrimitiveType::Edge);
    int boundary_loop_id = 0;
    for (auto edge_tuple : all_edge_tuples) {
        if (!m.is_boundary_edge(edge_tuple)) {
            continue;
        }
        const long eid = m.id(edge_tuple, PrimitiveType::Edge);
        if (boundary_loop_ids[eid] != -1) {
            continue;
        } // visited

        Tuple cur_edge = edge_tuple;
        long cur_eid = m.id(cur_edge, PrimitiveType::Edge);
        // find one boundary loop
        while (boundary_loop_ids[eid] == -1 || cur_eid != eid) {
            boundary_loop_ids[cur_eid] = boundary_loop_id;

            // find next boundary edge
            cur_edge = m.switch_edge(m.switch_vertex(cur_edge));
            while (!m.is_boundary_edge(cur_edge)) {
                cur_edge = m.switch_edge(m.switch_face(cur_edge));
            }

            cur_eid = m.id(cur_edge, PrimitiveType::Edge);
        }

        boundary_loop_id++;
    }

    return boundary_loop_id;
}

int trimesh_euler_characteristic(const DEBUG_TriMesh& m)
{
    auto simplex_counts = trimesh_simplex_counts(m);
    int boundary_loops_count = trimesh_boundary_loops_count(m);
    return simplex_counts[0] - simplex_counts[1] + simplex_counts[2] + boundary_loops_count;
}

int trimesh_genus(const DEBUG_TriMesh& m)
{
    if (trimesh_simply_connected_components(m) != 1) {
        wmtk::logger().error("not a connected surface\n");
        throw std::runtime_error("GenusComputeError");
    }
    return (2 - trimesh_euler_characteristic(m)) / 2;
}


void run_debug_trimesh(const DEBUG_TriMesh& m, const MeshDebugInfo& info)
{
    fmt::print("Running run_debug_trimesh  on {}\n", info.name);

    // validate that the info is ok
    REQUIRE(info.boundary_curves >= 0);
    REQUIRE(info.simply_connected_components >= 0);
    for (const long count : info.simplex_counts) {
        REQUIRE(count >= 0);
    }

    REQUIRE(m.is_connectivity_valid());
    if (info.genus >= 0) {
        CHECK(trimesh_genus(m) == info.genus);
    }
    CHECK(trimesh_simply_connected_components(m) == info.simply_connected_components);
    CHECK(trimesh_simplex_counts(m) == info.simplex_counts);
    REQUIRE(trimesh_boundary_loops_count(m) == info.boundary_curves);
    auto [v_count, e_count, f_count] = info.simplex_counts;
    auto v_tups = m.get_all(PrimitiveType::Vertex);
    auto e_tups = m.get_all(PrimitiveType::Edge);
    auto f_tups = m.get_all(PrimitiveType::Face);

    REQUIRE(size_t(v_count) == v_tups.size());
    REQUIRE(size_t(e_count) == e_tups.size());
    REQUIRE(size_t(f_count) == f_tups.size());
    for (const auto& v_tup : v_tups) {
        fmt::print("vertex {} is active\n", m.id(v_tup, PrimitiveType::Vertex));
    }
    for (const auto& e_tup : e_tups) {
        long id = m.id(e_tup, PrimitiveType::Edge);
        long a = m.id(e_tup, PrimitiveType::Vertex);
        long b = m.id(m.switch_tuple(e_tup, PrimitiveType::Vertex), PrimitiveType::Vertex);
        fmt::print("edge {} is active with vertices {} {}\n", id, a, b);
    }
    for (const auto& f_tup : f_tups) {
        long id = m.id(f_tup, PrimitiveType::Face);
        long a = m.id(f_tup, PrimitiveType::Vertex);
        long b = m.id(m.switch_tuple(f_tup, PrimitiveType::Vertex), PrimitiveType::Vertex);
        long c = m.id(
            m.switch_tuple(m.switch_tuple(f_tup, PrimitiveType::Edge), PrimitiveType::Vertex),
            PrimitiveType::Vertex);
        fmt::print("face {} is active with vertices {} {} {}\n", id, a, b, c);
    }
}
} // namespace

TEST_CASE("test_debug_trimeshes_single_triangle")
{
    DEBUG_TriMesh m;
    m = single_triangle();
    // trimesh_genus(m);
    MeshDebugInfo info;
    REQUIRE(trimesh_boundary_loops_count(m) == 1);
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
    info.genus = -1; // not applicable here
    info.boundary_curves = 3; // each triangle is individual
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
    REQUIRE(trimesh_boundary_loops_count(m) == 1);
    info.name = "edge_region";
    info.genus = 0;
    info.boundary_curves = 1;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{10, 19, 10}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_two_components")
{
    DEBUG_TriMesh m;
    m = three_triangles_with_two_components();
    REQUIRE(trimesh_boundary_loops_count(m) == 2);
    REQUIRE(trimesh_simply_connected_components(m) == 2);
}

TEST_CASE("test_debug_trimeshes_hole")
{
    DEBUG_TriMesh m;
    m = nine_triangles_with_a_hole();
    MeshDebugInfo info;
    REQUIRE(trimesh_boundary_loops_count(m) == 2);
    info.name = "nine_triangles_with_a_hole";
    info.genus = 0;
    info.boundary_curves = 2;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{9, 18, 9}};
    run_debug_trimesh(m, info);
}
