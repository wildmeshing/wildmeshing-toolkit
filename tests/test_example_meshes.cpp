
#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include <wmtk/utils/Logger.hpp>
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

std::array<long, 3> trimesh_simplex_counts(const DEBUG_TriMesh& m)
{
    return std::array<long, 3>{{long(m.get_all(PrimitiveType::Vertex).size()),
                                long(m.get_all(PrimitiveType::Edge).size()),
                                long(m.get_all(PrimitiveType::Face).size())}};
}


int trimesh_simply_connected_components(const DEBUG_TriMesh& m, std::vector<int> &component_ids)
{
    component_ids.resize(m.capacity(PrimitiveType::Face), -1);
    auto all_face_tuples = m.get_all(PrimitiveType::Face);
    int component_id = 0;
    // BFS
    for (size_t i = 0; i < all_face_tuples.size(); ++i) {
        long fid = m.id(all_face_tuples[i], PrimitiveType::Face);
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
            for (int j = 0; j < 3; ++j)
            {
                if (!m.is_boundary(f_tuple))
                {   
                    auto adj_face_tuple = m.switch_tuple(f_tuple, PrimitiveType::Face);
                    long adj_fid = m.id(adj_face_tuple, PrimitiveType::Face);
                    if (component_ids[adj_fid] == -1) {
                        q.push(adj_fid);
                    }  
                }
                f_tuple = m.switch_tuple(m.switch_tuple(f_tuple, PrimitiveType::Vertex), PrimitiveType::Edge);
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

int trimesh_n_bd_loops(const DEBUG_TriMesh& m)
{
    std::vector<int> bd_loops(m.capacity(PrimitiveType::Edge), -1);
    auto all_edge_tuples = m.get_all(PrimitiveType::Edge);
    int bd_loop_id = 0;
    for (size_t i = 0; i < all_edge_tuples.size(); ++i) 
    {
        if (!m.is_boundary(all_edge_tuples[i])) {
            continue;
        }
        long eid = m.id(all_edge_tuples[i], PrimitiveType::Edge);
        if (bd_loops[eid] != -1) {
            continue;
        } // visited

        Tuple cur_edge = all_edge_tuples[i];
        long cur_eid = m.id(cur_edge, PrimitiveType::Edge);
        // find one boundary loop
        while (bd_loops[m.id(all_edge_tuples[i], PrimitiveType::Edge)] == -1 || cur_eid != m.id(all_edge_tuples[i], PrimitiveType::Edge)) 
        {
            bd_loops[cur_eid] = bd_loop_id;

            // find next boundary edge
            cur_edge = m.switch_tuple(cur_edge, PrimitiveType::Vertex);
            cur_edge = m.switch_tuple(cur_edge, PrimitiveType::Edge);
            while(!m.is_boundary(cur_edge))
            {
                cur_edge = m.switch_tuple(cur_edge, PrimitiveType::Face);
                cur_edge = m.switch_tuple(cur_edge, PrimitiveType::Edge);
            }
            
            cur_eid = m.id(cur_edge, PrimitiveType::Edge);
        }
        
        bd_loop_id++;
    }

    return bd_loop_id;
}

int trimesh_euler_char(const DEBUG_TriMesh& m)
{
    auto simplex_counts = trimesh_simplex_counts(m);
    int n_bd_loops = trimesh_n_bd_loops(m);
    return simplex_counts[0] - simplex_counts[1] + simplex_counts[2] + n_bd_loops;
}

int trimesh_genus(const DEBUG_TriMesh& m)
{
    if (trimesh_simply_connected_components(m) != 1)
    {
        wmtk::logger().error("not a connected surface\n");
        throw std::runtime_error("GenusComputeError");
    }
    return (2 - trimesh_euler_char(m)) / 2;
}



void run_debug_trimesh(const DEBUG_TriMesh& m, const MeshDebugInfo& info)
{
    // validate that the info is ok
    REQUIRE(info.genus >= 0);
    REQUIRE(info.simply_connected_components>= 0);
    for(const long count: info.simplex_counts) {
        REQUIRE(count >= 0);
    }

    REQUIRE(m.is_connectivity_valid());

    CHECK(trimesh_genus(m) == info.genus);
    CHECK(trimesh_simply_connected_components(m) == info.simply_connected_components);
    CHECK(trimesh_simplex_counts(m) == info.simplex_counts);

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
    // trimesh_genus(m);
    MeshDebugInfo info;
    REQUIRE(trimesh_n_bd_loops(m) == 1);
    info.name = "single_triangle";
    info.genus = 0;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{3, 3, 1}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_one_ear")
{
    DEBUG_TriMesh m;
    m = one_ear();
    MeshDebugInfo info;
    REQUIRE(trimesh_n_bd_loops(m) == 1);
    info.name = "one_ear";
    info.genus = 0;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{4, 5, 2}};
    run_debug_trimesh(m, info);
}

TEST_CASE("test_debug_trimeshes_two_components")
{
    DEBUG_TriMesh m;
    m = three_triangles_with_two_components();
    MeshDebugInfo info;
    REQUIRE(trimesh_n_bd_loops(m) == 2);
    REQUIRE(trimesh_simply_connected_components(m) == 2);
}

TEST_CASE("test_debug_trimeshes_hole")
{
    DEBUG_TriMesh m;
    m = nine_triangles_with_a_hole();
    MeshDebugInfo info;
    REQUIRE(trimesh_n_bd_loops(m) == 2);
    info.name = "nine_triangles_with_a_hole";
    info.genus = 0;
    info.simply_connected_components = 1;
    info.simplex_counts = std::array<long, 3>{{9, 18, 9}};
    run_debug_trimesh(m, info);
}