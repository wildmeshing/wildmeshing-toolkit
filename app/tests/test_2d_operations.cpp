#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <catch2/catch.hpp>
#include <iostream>
#include "EdgeCollapse/EdgeCollapse.h"

using namespace wmtk;


TEST_CASE("shortest_edge_collapse", "[test_2d_operations]")
{
    using namespace Edge2d;
    // 0___1___2    0 __1___2
    // \  /\  /      \  |  /
    // 3\/__\/4  ==>  \ | /
    //   \  /          \|/6
    //    \/5


    std::vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(3, 3, 0);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);
    v_positions[4] = Eigen::Vector3d(0.5, 0, 0);
    v_positions[5] = Eigen::Vector3d(0, -3, 0);
    EdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}, {{3, 4, 5}}};
    m.create_mesh(6, tris);
    std::vector<TriMesh::Tuple> edges = m.get_edges();
    // find the shortest edge
    double shortest = std::numeric_limits<double>::max();
    TriMesh::Tuple shortest_edge;
    for (TriMesh::Tuple t : edges) {
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