
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

std::array<long, 3> trimesh_simplex_counts(const DEBUG_TriMesh& m)
{
    return std::array<long, 3>{{m.get_all(PrimitiveType::Vertex).size(),
                                 m.get_all(PrimitiveType::Edge).size(),
                                 m.get_all(PrimitiveType::Face).size()}};
}


int trimesh_simply_connected_components(const DEBUG_TriMesh& m, std::vector<int> &component_ids)
{
    component_ids.resize(m.capacity(PrimitiveType::Face), -1);
    auto all_face_tuples = m.get_all(PrimitiveType::Face);
    int component_id = 0;
    // BFS
    for (int i = 0; i < all_face_tuples.size(); ++i) {
        int fid = m.id(all_face_tuples[i], PrimitiveType::Face);
        if (component_ids[fid] != -1) {
            continue;
        } // visited

        std::queue<int> q;
        q.push(fid);
        while (!q.empty()) {
            int cur_fid = q.front();
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


int trimesh_genus(const DEBUG_TriMesh& m)
{
    auto flag_f_accessor = m.get_flag_accessor(PrimitiveType::Face);
    std::cout << flag_f_accessor.scalar_attribute(m.tuple_from_face_id(0)) << std::endl;
    return 0;
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

    // TODO: in the future we should check for some topological info
    // CHECK(genus(m) == info.genus);
    CHECK(trimesh_simply_connected_components(m) == info.simply_connected_components);
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
    // trimesh_genus(m);
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
