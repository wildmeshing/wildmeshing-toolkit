#include <stdlib.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <catch2/catch.hpp>
#include <iostream>
#include "ParallelEdgeCollapse/ParallelEdgeCollapse.h"
#include <tbb/concurrent_vector.h>

using namespace wmtk;


TEST_CASE("parallel_shortest_edge_collapse", "[test_2d_parallel_operations]")
{
    using namespace Edge2d;
    // 0___1___2    0 __1___2
    // \  /\  /      \  |  /
    // 3\/__\/4  ==>  \ | /
    //   \  /          \|/6
    //    \/5


    tbb::concurrent_vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(3, 3, 0);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);
    v_positions[4] = Eigen::Vector3d(0.5, 0, 0);
    v_positions[5] = Eigen::Vector3d(0, -3, 0);
    ParallelEdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}, {{3, 4, 5}}};
    m.create_mesh(6, tris);
    std::vector<ConcurrentTriMesh::Tuple> edges = m.get_edges();
    // find the shortest edge
    double shortest = std::numeric_limits<double>::max();
    ConcurrentTriMesh::Tuple shortest_edge;
    for (ConcurrentTriMesh::Tuple t : edges) {
        size_t v1 = t.get_vid();
        size_t v2 = m.switch_vertex(t).get_vid();
        if ((v_positions[v1] - v_positions[v2]).squaredNorm() < shortest) {
            shortest = (v_positions[v1] - v_positions[v2]).squaredNorm();
            shortest_edge = t;
        }
    }
    m.collapse_shortest();
    // the collapsed edge tuple is not valid anymore
    REQUIRE_FALSE(shortest_edge.is_valid(m));
}