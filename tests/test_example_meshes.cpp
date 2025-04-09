
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
    int64_t boundary_curves = -1; // number of distinct boundary curves in the mesh
    int64_t genus = -1; // TODO (also what definition of genus?)
    int64_t simply_connected_components = -1; // TODO (face-face connected topologies)
    bool is_oriented = false; // TODO (make sure nface neighbors use opposite orientation
    // the number of simplices (vertex, edge, then face)
    std::array<int64_t, 3> simplex_counts = std::array<int64_t, 3>{{-1, -1, -1}};
};

std::array<int64_t, 3> trimesh_simplex_counts(const TriMesh& m)
{
    return std::array<int64_t, 3>{
        {int64_t(m.get_all(PrimitiveType::Vertex).size()),
         int64_t(m.get_all(PrimitiveType::Edge).size()),
         int64_t(m.get_all(PrimitiveType::Triangle).size())}};
}


/***
 * @brief compute the number of simply connected components in a mesh
 * @param m the mesh
 * @param component_ids output the component id of each face
 * @return the number of simply connected components
 */

int trimesh_simply_connected_components(const DEBUG_TriMesh& m, std::vector<int>& component_ids)
{
    component_ids.resize(m.capacity(PrimitiveType::Triangle), -1);
    const auto all_face_tuples = m.get_all(PrimitiveType::Triangle);
    int component_id = 0;
    // BFS
    for (auto face_tuple : all_face_tuples) {
        const int64_t fid = m.id(face_tuple, PrimitiveType::Triangle);
        if (component_ids[fid] != -1) {
            continue;
        } // visited

        std::queue<int64_t> q;
        q.push(fid);
        while (!q.empty()) {
            int64_t cur_fid = q.front();
            q.pop();

            component_ids[cur_fid] = component_id;
            auto f_tuple = m.tuple_from_face_id(cur_fid);
            // push all adjacent faces
            for (int j = 0; j < 3; ++j) {
                if (!m.is_boundary_edge(f_tuple)) {
                    auto adj_face_tuple = m.switch_tuple(f_tuple, PrimitiveType::Triangle);
                    int64_t adj_fid = m.id(adj_face_tuple, PrimitiveType::Triangle);
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
        const int64_t eid = m.id(edge_tuple, PrimitiveType::Edge);
        if (boundary_loop_ids[eid] != -1) {
            continue;
        } // visited

        Tuple cur_edge = edge_tuple;
        int64_t cur_eid = m.id(cur_edge, PrimitiveType::Edge);
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
    // fmt::print("Running run_debug_trimesh  on {}\n", info.name);

    // validate that the info is ok
    REQUIRE(info.boundary_curves >= 0);
    REQUIRE(info.simply_connected_components >= 0);
    for (const int64_t count : info.simplex_counts) {
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
    auto f_tups = m.get_all(PrimitiveType::Triangle);

    REQUIRE(size_t(v_count) == v_tups.size());
    REQUIRE(size_t(e_count) == e_tups.size());
    REQUIRE(size_t(f_count) == f_tups.size());
    // for (const auto& v_tup : v_tups) {
    //     fmt::print("vertex {} is active\n", m.id(v_tup, PrimitiveType::Vertex));
    // }
    for (const auto& e_tup : e_tups) {
        int64_t id = m.id(e_tup, PrimitiveType::Edge);
        int64_t a = m.id(e_tup, PrimitiveType::Vertex);
        int64_t b = m.id(m.switch_tuple(e_tup, PrimitiveType::Vertex), PrimitiveType::Vertex);
        // fmt::print("edge {} is active with vertices {} {}\n", id, a, b);
    }
    for (const auto& f_tup : f_tups) {
        int64_t id = m.id(f_tup, PrimitiveType::Triangle);
        int64_t a = m.id(f_tup, PrimitiveType::Vertex);
        int64_t b = m.id(m.switch_tuple(f_tup, PrimitiveType::Vertex), PrimitiveType::Vertex);
        int64_t c = m.id(
            m.switch_tuple(m.switch_tuple(f_tup, PrimitiveType::Edge), PrimitiveType::Vertex),
            PrimitiveType::Vertex);
        // fmt::print("face {} is active with vertices {} {} {}\n", id, a, b, c);
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
    info.simplex_counts = std::array<int64_t, 3>{{3, 3, 1}};
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
    info.simplex_counts = std::array<int64_t, 3>{{4, 5, 2}};
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
    info.simplex_counts = std::array<int64_t, 3>{{5, 7, 3}};
    run_debug_trimesh(m, info);


    auto tups = m.get_all(wmtk::PrimitiveType::Triangle);


    const auto& fv_acc = *m.m_fv_accessor;
    const auto& fe_acc = *m.m_fe_accessor;
    const auto& ff_acc = *m.m_ff_accessor;

    CHECK(fv_acc.const_vector_attribute(tups[0]) == Vector3l(0, 1, 2));
    CHECK(fv_acc.const_vector_attribute(tups[1]) == Vector3l(3, 1, 0));
    CHECK(fv_acc.const_vector_attribute(tups[2]) == Vector3l(0, 2, 4));

    CHECK(fe_acc.const_vector_attribute(tups[0]) == Vector3l(4, 1, 0));
    CHECK(fe_acc.const_vector_attribute(tups[1]) == Vector3l(0, 2, 5));
    CHECK(fe_acc.const_vector_attribute(tups[2]) == Vector3l(6, 3, 1));

    CHECK(ff_acc.const_vector_attribute(tups[0]) == Vector3l(-1, 2, 1));
    CHECK(ff_acc.const_vector_attribute(tups[1]) == Vector3l(0, -1, -1));
    CHECK(ff_acc.const_vector_attribute(tups[2]) == Vector3l(-1, -1, 0));
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
    info.simplex_counts = std::array<int64_t, 3>{{6, 9, 4}};
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
    info.simplex_counts = std::array<int64_t, 3>{{6, 9, 3}};
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
    info.simplex_counts = std::array<int64_t, 3>{{4, 6, 4}};
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
    info.simplex_counts = std::array<int64_t, 3>{{9, 16, 8}};
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
    info.simplex_counts = std::array<int64_t, 3>{{10, 19, 10}};
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
    info.simplex_counts = std::array<int64_t, 3>{{9, 18, 9}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_disk_trimesh")
{
    // TODO: N=1 should be possible, but have to fix teh trimesh impl
    for (int N = 2; N < 10; ++N) {
        auto mptr = std::static_pointer_cast<DEBUG_TriMesh>(disk(N));
        auto& m = *mptr;
        MeshDebugInfo info;
        info.name = fmt::format("disk_{}", N);
        info.genus = 0;
        info.simply_connected_components = 1;
        info.boundary_curves = 1;
        int64_t vcount = N + 1;
        int64_t ecount = 2 * N;
        int64_t fcount = N;
        if (N == 2) { // this one gets registered as a closed mesh due to id-based merging
            info.boundary_curves = 0;
            ecount = 3;
            REQUIRE(trimesh_boundary_loops_count(m) == 0);
        } else {
            REQUIRE(trimesh_boundary_loops_count(m) == 1);
        }
        info.simplex_counts = std::array<int64_t, 3>{{vcount, ecount, fcount}};
        run_debug_trimesh(m, info);
    }

    {
        // the 0,1,1 mesh
        auto mptr = std::static_pointer_cast<DEBUG_TriMesh>(disk(1));
        auto& m = *mptr;
        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PrimitiveType::Vertex));
        auto fv = fv_accessor.vector_attribute(0);
        REQUIRE(fv(0) == 0);
        REQUIRE(fv(1) == 1);
        REQUIRE(fv(1) == 1);

        // the edge 1,1 is a boundary edge by our convention
        size_t n_boundary_edges = 0;
        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            if (m.is_boundary_edge(e)) {
                ++n_boundary_edges;
            }
        }
        CHECK(n_boundary_edges == 1);

        // the vertex 1 is on the boundary of 0,0 as a side effect of edge 1,1
        size_t n_boundary_vertices = 0;
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            if (m.is_boundary_vertex(v)) {
                ++n_boundary_vertices;
            }
        }
        CHECK(n_boundary_vertices == 1);


        Tuple v0e1(0, 1, -1, 0);
        Tuple v0e2(0, 2, -1, 0);
        Tuple v1e0(1, 0, -1, 0);
        Tuple v1e2(1, 2, -1, 0);
        Tuple v2e0(2, 0, -1, 0);
        Tuple v2e1(2, 1, -1, 0);


        CHECK(m.switch_face(v0e1) == v0e2);
        CHECK(m.switch_face(v0e2) == v0e1);

        CHECK(m.is_boundary_edge(v1e0));
        CHECK(m.is_boundary_edge(v2e0));

        CHECK(m.switch_face(v1e2) == v2e1);
        CHECK(m.switch_face(v2e1) == v1e2);
    }
}
TEST_CASE("test_debug_individual_trimesh")
{
    for (int N = 1; N < 10; ++N) {
        auto mptr = std::static_pointer_cast<DEBUG_TriMesh>(individual_triangles(N));
        auto& m = *mptr;
        MeshDebugInfo info;
        info.name = fmt::format("individual_{}", N);
        info.genus = -1; // TODO genus stuff doesnt work for now
        info.simply_connected_components = N;
        info.boundary_curves = N;
        int64_t vcount = 3 * N;
        int64_t ecount = 3 * N;
        int64_t fcount = N;
        info.simplex_counts = std::array<int64_t, 3>{{vcount, ecount, fcount}};
        run_debug_trimesh(m, info);
    }
}
