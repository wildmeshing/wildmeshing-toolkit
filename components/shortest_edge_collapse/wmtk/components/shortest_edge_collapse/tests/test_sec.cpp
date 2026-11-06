#include <wmtk/utils/Logger.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/read_triangle_mesh.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>

using namespace wmtk;
using namespace components::shortest_edge_collapse;

namespace {
// The octahedron of the unit vectors, its faces facing out, with vertex 4 (+z)
// moved to (0.5, 0, 0.6): its edge to vertex 0 (+x), 0.78 long, is the shortest
// (the next, 4 to 2 and 4 to 3, are 1.27). A closed mesh, so freeze_boundary()
// freezes no vertex and every collapse the link condition allows can happen.
// Until 2026-11-05 these cases ran on open patches, every vertex of which lies
// on the boundary, is frozen, and no collapse could succeed.
struct Octahedron
{
    std::vector<Eigen::Vector3d> v = {
        Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d(-1, 0, 0),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0.5, 0, 0.6),
        Eigen::Vector3d(0, 0, -1)};
    std::vector<std::array<size_t, 3>> tris = {
        {{0, 2, 4}},
        {{1, 4, 2}},
        {{0, 4, 3}},
        {{0, 5, 2}},
        {{1, 3, 4}},
        {{1, 2, 5}},
        {{0, 3, 5}},
        {{1, 5, 3}}};
};
// The edge of `m` between vertices a and b.
TriMesh::Tuple edge_between(const ShortestEdgeCollapse& m, size_t a, size_t b)
{
    for (const TriMesh::Tuple& e : m.get_edges()) {
        const size_t u = e.vid(m), w = e.switch_vertex(m).vid(m);
        if ((u == a && w == b) || (u == b && w == a)) return e;
    }
    FAIL("no edge " << a << "-" << b);
    return {};
}
} // namespace

TEST_CASE("shortest_edge_collapse", "[test_sec]")
{
    // Collapsing every edge it can, shortest first, takes the octahedron down to
    // a tet, the shortest edge among the collapsed.
    const Octahedron o;
    ShortestEdgeCollapse m(o.v);
    m.create_mesh(o.v.size(), o.tris);
    const TriMesh::Tuple shortest = edge_between(m, 0, 4);
    REQUIRE(m.check_link_condition(shortest));
    m.collapse_shortest(-1);
    REQUIRE_FALSE(shortest.is_valid(m));
    REQUIRE(m.get_vertices().size() == 4);
    REQUIRE(m.get_faces().size() == 4);
}

TEST_CASE("shortest_edge_collapse_shortest_first", "[test_sec]")
{
    // One collapse (target 5 vertices of 6): the shortest edge's, onto its
    // midpoint (no endpoint frozen or on a boundary, collapse_edge_after()). The
    // vertices left are 1, 2, 3 and 5 where they were and the midpoint of 0 and
    // 4; a collapse of any other edge at 0 or 4 would leave another set.
    const Octahedron o;
    ShortestEdgeCollapse m(o.v);
    m.create_mesh(o.v.size(), o.tris);
    m.collapse_shortest(5);
    REQUIRE(m.get_vertices().size() == 5);
    REQUIRE(m.get_faces().size() == 6);
    REQUIRE(m.check_mesh_connectivity_validity());
    std::vector<std::array<double, 3>> left, expected;
    for (const TriMesh::Tuple& t : m.get_vertices()) {
        const Eigen::Vector3d& p = m.vertex_attrs[t.vid(m)].pos;
        left.push_back({{p[0], p[1], p[2]}});
    }
    for (const size_t i : {1, 2, 3, 5}) expected.push_back({{o.v[i][0], o.v[i][1], o.v[i][2]}});
    const Eigen::Vector3d mid = (o.v[0] + o.v[4]) / 2.0;
    expected.push_back({{mid[0], mid[1], mid[2]}});
    std::sort(left.begin(), left.end());
    std::sort(expected.begin(), expected.end());
    CHECK(left == expected);
}

TEST_CASE("shortest_edge_collapse_closed_mesh", "[test_sec]")
{
    SECTION("test on tet")
    {
        // create a tet and collapse can't happen
        std::vector<Eigen::Vector3d> v_positions(6);
        v_positions[0] = Eigen::Vector3d(-3, 3, 0);
        v_positions[1] = Eigen::Vector3d(0, 3, 0);
        v_positions[2] = Eigen::Vector3d(0, 0, 2);
        v_positions[3] = Eigen::Vector3d(0, 0, 0);

        ShortestEdgeCollapse m(v_positions);
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
        const std::string root(WMTK_DATA_DIR);
        const std::string path = root + "/models/piece_0.obj";

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
        ShortestEdgeCollapse m(v);
        m.create_mesh(V.rows(), tri);
        REQUIRE(m.check_mesh_connectivity_validity());
        REQUIRE(m.collapse_shortest(100));

        std::vector<TriMesh::Tuple> edges = m.get_edges();
        REQUIRE(m.get_vertices().size() == 4);

        REQUIRE(m.get_faces().size() == 4);
    }
}

TEST_CASE("shortest_edge_collapse_octocat", "[test_sec][.slow]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/Octocat.obj";

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
    ShortestEdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.collapse_shortest(2000));
    REQUIRE(m.check_mesh_connectivity_validity());
}

TEST_CASE("edge_manifold", "[test_sec]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/circle.obj";

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
    ShortestEdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(igl::is_edge_manifold(F) == m.check_edge_manifold());
}

TEST_CASE("shortest_edge_collapse_circle", "[test_sec]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/circle.obj";

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
    ShortestEdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(igl::is_edge_manifold(F));
    REQUIRE(m.check_edge_manifold());
    REQUIRE(m.collapse_shortest(100));
    m.consolidate_mesh();
    m.write_triangle_mesh("collapsed.obj");
}

TEST_CASE("metis_test_bigmesh", "[test_sec][.slow]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/circle.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    REQUIRE(ok);

    // change this for max concurrency
    int max_num_threads = 8;

    std::vector<double> timecost;

    for (int thread = 1; thread <= max_num_threads; thread *= 2) {
        std::vector<Eigen::Vector3d> v(V.rows());
        std::vector<std::array<size_t, 3>> tri(F.rows());
        for (int i = 0; i < V.rows(); i++) {
            v[i] = V.row(i);
        }
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
        }

        ShortestEdgeCollapse m(v, thread);
        // m.print_num_attributes();
        m.create_mesh(V.rows(), tri);
        REQUIRE(m.check_mesh_connectivity_validity());
        igl::Timer timer;
        double time;
        timer.start();

        // change this for num of operations
        m.collapse_shortest(20);
        time = timer.getElapsedTimeInMilliSec();
        timecost.push_back(time);
    }
}
TEST_CASE("sec-holds-each-face-to-its-own-input", "[test_sec]")
{
    // Input 0: a square tent, its apex 3 eps above the flat square of input 1.
    // The apex is the one vertex not on a boundary, so the only collapses move it
    // onto a corner, which lays input 0 flat on input 1, 2 eps from itself at a
    // remaining face's centroid. Held to the union that is inside; held to its
    // own input's envelope it is not, and the apex stays. Without
    // per_input_envelopes the inputs share one envelope again, whatever tri_input
    // says.
    const double eps = 1e-2;
    const std::vector<Eigen::Vector3d> v = {
        {0, 0, 0},
        {1, 0, 0},
        {1, 1, 0},
        {0, 1, 0},
        {0.5, 0.5, 3 * eps},
        {0, 0, 0},
        {1, 0, 0},
        {1, 1, 0},
        {0, 1, 0}};
    const std::vector<std::array<size_t, 3>> tris =
        {{{0, 1, 4}}, {{1, 2, 4}}, {{2, 3, 4}}, {{3, 0, 4}}, {{5, 6, 7}}, {{5, 7, 8}}};
    const auto n_vertices_after = [&](const std::vector<int>& tri_input, bool per_input = true) {
        ShortestEdgeCollapse m(v, 0, true);
        m.create_mesh(v.size(), tris, {}, eps, 0, tri_input, per_input);
        m.collapse_shortest(0);
        return m.get_vertices().size();
    };
    CHECK(n_vertices_after({}) == 8); // the control: the union lets the apex go
    CHECK(n_vertices_after({0, 0, 0, 0, 1, 1}) == 9);
    CHECK(n_vertices_after({0, 0, 0, 0, 1, 1}, false) == 8);
}
