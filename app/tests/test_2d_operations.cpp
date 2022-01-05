#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <catch2/catch.hpp>
#include <iostream>
#include "EdgeCollapse/EdgeCollapse.h"
using namespace wmtk;

using namespace Edge2d;
TEST_CASE("shortest_edge_collapse", "[test_2d_operations]")
{
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
    m.collapse_shortest(100);
    // the collapsed edge tuple is not valid anymore
    REQUIRE_FALSE(shortest_edge.is_valid(m));
}

TEST_CASE("shortest_edge_collapse_boundary_edge", "[test_2d_operations]")
{
    // 0___1___2    0 __1___2   0 __1
    // \  /\  /      \  |  /     \  |
    // 3\/__\/4  ==>  \ | / ==>   \ |
    //                 \|/5        \|6
    //

    std::vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(3, 3, 0);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);
    v_positions[4] = Eigen::Vector3d(0.5, 0, 0);
    EdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}};
    m.create_mesh(5, tris);
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
    m.collapse_shortest(100);
    // the collapsed edge tuple is not valid anymore
    REQUIRE_FALSE(shortest_edge.is_valid(m));
    m.write_triangle_mesh("/Users/yunfanzhou/Downloads/tmp/collapsed.obj");
}

TEST_CASE("shortest_edge_collapse_closed_mesh", "[test_2d_operations]")
{
    // create a tet
    std::vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(0, 0, 2);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);

    EdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 3}}, {{0, 3, 2}}, {{0, 1, 2}}};
    m.create_mesh(4, tris);
    std::vector<TriMesh::Tuple> edges = m.get_edges();
    m.collapse_shortest(100);
    m.compact();
    REQUIRE(m.n_vertices() == 3);

    REQUIRE(m.n_triangles() == 2);
}


TEST_CASE("shortest_edge_collapse_on_mesh", "[test_2d_operations]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/piece_0.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    REQUIRE(V.rows() == 8);
    REQUIRE(F.rows() == 12);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
        // std::cout << V.row(i) << std::endl;
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    std::cout << " is it mesh passed " << std ::endl;
    REQUIRE(m.collapse_shortest(5));
    m.write_triangle_mesh("collapsed.obj");
}


TEST_CASE("shortest_edge_collapse_octocat", "[test_2d_operations]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/Octocat.obj";
    
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
        // std::cout << V.row(i) << std::endl;
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    std::cout << " is it mesh passed " << std ::endl;
    REQUIRE(m.collapse_shortest(1000));
    m.write_triangle_mesh("collapsed.obj");
}
