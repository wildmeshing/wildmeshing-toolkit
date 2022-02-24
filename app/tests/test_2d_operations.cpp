#include <EdgeOperations2d.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <catch2/catch.hpp>
#include <iostream>
using namespace wmtk;

using namespace Edge2d;
TEST_CASE("shortest_edge_collapse", "[test_2d_operations]")
{
    // 0___1___2    0___1       *
    // \  /\  /      \  /\      *
    // 3\/__\/4  ==> 3\/__\6    *
    //   \  /          \  /     *
    //    \/5           \/5     *
    // 3-4 is shortest

    std::vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(3, 3, 0);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);
    v_positions[4] = Eigen::Vector3d(0.5, 0, 0);
    v_positions[5] = Eigen::Vector3d(0, -3, 0);
    EdgeOperations2d m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}, {{3, 4, 5}}};
    m.create_mesh(6, tris);
    std::vector<TriMesh::Tuple> edges = m.get_edges();
    // find the shortest edge
    double shortest = std::numeric_limits<double>::max();
    TriMesh::Tuple shortest_edge;
    for (TriMesh::Tuple t : edges) {
        size_t v1 = t.vid(m);
        size_t v2 = m.switch_vertex(t).vid(m);
        if ((v_positions[v1] - v_positions[v2]).squaredNorm() < shortest) {
            shortest = (v_positions[v1] - v_positions[v2]).squaredNorm();
            shortest_edge = t;
        }
    }

    REQUIRE_FALSE(m.check_link_condition(shortest_edge));
    REQUIRE(m.collapse_shortest(1));
    REQUIRE_FALSE(shortest_edge.is_valid(m));

    m.consolidate_mesh();

    REQUIRE(m.get_vertices().size() == 3);
    REQUIRE(m.get_faces().size() == 1);
}

TEST_CASE("shortest_edge_collapse_boundary_edge", "[test_2d_operations]")
{
    // 0___1___2    0 __1___2      0 __1
    // \  /\  /      \  |  /         \ |
    // 3\/__\/4  ==>  \ | / ==>        6
    //                 \|/5
    //

    std::vector<Eigen::Vector3d> v_positions(6);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);
    v_positions[2] = Eigen::Vector3d(3, 3, 0);
    v_positions[3] = Eigen::Vector3d(0, 0, 0);
    v_positions[4] = Eigen::Vector3d(0.5, 0, 0);
    EdgeOperations2d m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}};
    m.create_mesh(5, tris);
    std::vector<TriMesh::Tuple> edges = m.get_edges();
    // find the shortest edge
    double shortest = std::numeric_limits<double>::max();
    TriMesh::Tuple shortest_edge;
    for (TriMesh::Tuple t : edges) {
        size_t v1 = t.vid(m);
        size_t v2 = m.switch_vertex(t).vid(m);
        if ((v_positions[v1] - v_positions[v2]).squaredNorm() < shortest) {
            shortest = (v_positions[v1] - v_positions[v2]).squaredNorm();
            shortest_edge = t;
        }
    }
    m.collapse_shortest(100);
    // the collapsed edge tuple is not valid anymore
    REQUIRE_FALSE(shortest_edge.is_valid(m));

    m.write_triangle_mesh("collapsed.obj");
    REQUIRE(m.get_vertices().size() == 3);
    REQUIRE(m.get_faces().size() == 1);
}

TEST_CASE("shortest_edge_collapse_closed_mesh", "[test_2d_operations]")
{
    SECTION("test on tet")
    {
        // create a tet and collapse can't happen
        std::vector<Eigen::Vector3d> v_positions(6);
        v_positions[0] = Eigen::Vector3d(-3, 3, 0);
        v_positions[1] = Eigen::Vector3d(0, 3, 0);
        v_positions[2] = Eigen::Vector3d(0, 0, 2);
        v_positions[3] = Eigen::Vector3d(0, 0, 0);

        EdgeOperations2d m(v_positions);
        std::vector<std::array<size_t, 3>> tris = {
            {{0, 1, 3}},
            {{1, 2, 3}},
            {{0, 3, 2}},
            {{0, 1, 2}}};
        m.create_mesh(4, tris);
        m.collapse_shortest(100);
        REQUIRE(m.vert_capacity() == 4);

        REQUIRE(m.tri_capacity() == 4);
    }
    SECTION("test on cube, end with tet")
    {
        // then test on a cube
        // will have a tet in the end
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
        }
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
        }
        EdgeOperations2d m(v);
        m.create_mesh(V.rows(), tri);
        REQUIRE(m.check_mesh_connectivity_validity());
        REQUIRE(m.collapse_shortest(100));

        std::vector<TriMesh::Tuple> edges = m.get_edges();
        REQUIRE(m.get_vertices().size() == 4);

        REQUIRE(m.get_faces().size() == 4);
    }
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
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.collapse_shortest(50));
    REQUIRE(m.check_mesh_connectivity_validity());
}

TEST_CASE("shortest_edge_collapse_circle", "[test_2d_operations]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/circle.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.collapse_shortest(100));
    m.write_triangle_mesh("collapsed.obj");
}

TEST_CASE("test_swap", "[test_2d_operations]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/circle.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    int v_invariant = m.get_vertices().size();
    int e_invariant = m.get_edges().size();
    REQUIRE(m.check_mesh_connectivity_validity());
    auto edges = m.get_edges();
    std::vector<TriMesh::Tuple> new_e;
    int cnt = 0;
    for (auto edge : edges) {
        if (cnt > 200) break;
        if (!edge.is_valid(m)) continue;
        if (!(edge.switch_face(m)).has_value()) {
            REQUIRE_FALSE(m.swap_edge(edge, new_e));
            continue;
        }
        REQUIRE(m.swap_edge(edge, new_e));
        cnt++;
    }
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.get_vertices().size() == v_invariant);
    REQUIRE(m.get_edges().size() == e_invariant);
    // m.write_triangle_mesh("sawped.obj");
}

TEST_CASE("test_split", "[test_2d_operations]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/fan.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);

    auto edges = m.get_edges();
    for (auto edge : edges) {
        if (!edge.is_valid(m)) continue;
        std::vector<TriMesh::Tuple> dummy;
        m.split_edge(edge, dummy);
    }
    // m.write_triangle_mesh("split.obj");
    REQUIRE(m.check_mesh_connectivity_validity());
}

TEST_CASE("adaptive_remeshing", "[test_2d_operations][.]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/circle.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.adaptive_remeshing(0.01, 5, 1));
}

TEST_CASE("qec", "[test_2d_operations][.]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/piece_0.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("readin mesh is {}", ok);
    REQUIRE(ok);
    wmtk::logger().info("number of vertices {}", V.size());
    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.collapse_qec(V.rows() - 2));


    m.write_triangle_mesh("no_qec.obj");
}

TEST_CASE("split_each_edge", "[test_2d_operations]")
{
    std::vector<Eigen::Vector3d> v_positions(3);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);

    v_positions[2] = Eigen::Vector3d(0, 0, 0);

    EdgeOperations2d m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.create_mesh(3, tris);
    m.adaptive_remeshing(0.1, 5, 1);
}

TEST_CASE("qec_cost")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/upsample_box.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("readin mesh is {}", ok);
    REQUIRE(ok);
    wmtk::logger().info("number of vertices {}", V.size());
    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    auto edges = m.get_edges();
    Eigen::MatrixXd writem(edges.size(), 7);

    for (int i = 0; i < edges.size(); i++) {
        Eigen::Vector3d v1 = V.row(edges[i].vid(m));
        Eigen::Vector3d v2 = V.row(edges[i].switch_vertex(m).vid(m));
        writem.row(i) << v1.transpose(), v2.transpose(), m.compute_cost_for_e(edges[i]);
    }
}