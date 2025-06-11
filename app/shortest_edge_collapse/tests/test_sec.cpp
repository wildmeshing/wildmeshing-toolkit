#include <sec/ShortestEdgeCollapse.h>
#include <wmtk/TriMeshOperation.h>
#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>


using namespace app::sec;
using namespace wmtk;

TEST_CASE("separate-manifold-patch", "[test_sec]")
{
    std::vector<Eigen::Vector3d> v = {
        {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(0, -1, 0)}};
    std::vector<std::array<size_t, 3>> tris = {{
        {{0, 1, 2}},
        {{0, 1, 3}},
        {{0, 1, 4}},
    }};

    std::vector<Eigen::Vector3d> out_v;
    std::vector<std::array<size_t, 3>> out_f;
    std::vector<size_t> freeze_v;
    wmtk::separate_to_manifold(v, tris, out_v, out_f, freeze_v);
    REQUIRE(out_v.size() == 9);
    REQUIRE(out_f.size() == 3);

    // after extraction the output should be manifold
    Eigen::VectorXi dummy;
    Eigen::MatrixXd F(out_f.size(), 3);
    for (int i = 0; i < out_f.size(); i++) {
        F.row(i) << out_f[i][0], out_f[i][1], out_f[i][2];
    }
    REQUIRE(igl::is_edge_manifold(F));

    ShortestEdgeCollapse m(out_v);
    m.create_mesh(out_v.size(), out_f, freeze_v, 0);
    m.collapse_shortest(-1);
    Eigen::MatrixXi Fafter = Eigen::MatrixXi::Constant(m.tri_capacity(), 3, -1);
    for (auto& t : m.get_faces()) {
        auto i = t.fid(m);
        auto vs = m.oriented_tri_vertices(t);
        Fafter.row(i) << vs[0].vid(m), vs[1].vid(m), vs[2].vid(m);
    }
    // after operations the mesh is manifold
    REQUIRE(igl::is_edge_manifold(Fafter));
    REQUIRE(igl::is_vertex_manifold(Fafter, dummy));
}

TEST_CASE("manifold-separate-test-37989", "[test_sec]")
{
    std::string filename = WMTK_DATA_DIR "/37989_sf.obj";
    wmtk::manifold_internal::Vertices V;
    wmtk::manifold_internal::Facets F;
    igl::read_triangle_mesh(filename, V, F);
    REQUIRE_FALSE(igl::is_edge_manifold(F));
    std::vector<size_t> modified_vertices;
    wmtk::manifold_internal::resolve_nonmanifoldness(V, F, modified_vertices);
    REQUIRE(modified_vertices.size() > 0);
    REQUIRE(igl::is_edge_manifold(F));
    Eigen::VectorXi VI;
    REQUIRE(igl::is_vertex_manifold(F, VI));
}


TEST_CASE("shortest_edge_collapse", "[test_sec]")
{
    // 0___1___2                *
    // \  /\  /                 *
    // 3\/__\/4  ==>    __      *
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
    ShortestEdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{1, 4, 3}}, {{3, 4, 5}}};
    m.create_mesh_nofreeze(6, tris);
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

    REQUIRE_FALSE(TriMeshEdgeCollapseOperation::check_link_condition(m, shortest_edge));
    m.collapse_shortest(-1);

    REQUIRE_FALSE(shortest_edge.is_valid(m));

    REQUIRE(m.get_vertices().size() == 3);
    REQUIRE(m.get_faces().size() == 1);
}

TEST_CASE("shortest_edge_collapse_boundary_edge", "[test_sec]")
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
    ShortestEdgeCollapse m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 3}}, {{1, 2, 4}}, {{3, 1, 4}}};
    m.create_mesh_nofreeze(5, tris);
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
        m.create_mesh_nofreeze(4, tris);
        m.collapse_shortest(100);
        REQUIRE(m.vert_capacity() == 4);

        REQUIRE(m.tri_capacity() == 4);
    }
    SECTION("test on cube, end with tet")
    {
        // then test on a cube
        // will have a tet in the end
        const std::string root(WMTK_DATA_DIR);
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
    ShortestEdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.collapse_shortest(2000));
    REQUIRE(m.check_mesh_connectivity_validity());
}

TEST_CASE("edge_manifold", "[test_sec]")
{
    const std::string root(WMTK_DATA_DIR);
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
    ShortestEdgeCollapse m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(igl::is_edge_manifold(F) == m.check_edge_manifold());
}

TEST_CASE("shortest_edge_collapse_circle", "[test_sec]")
{
    const std::string root(WMTK_DATA_DIR);
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
    const std::string path = root + "/circle.obj";

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
