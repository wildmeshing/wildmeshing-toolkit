#include <TriWild.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>

using namespace wmtk;
using namespace triwild;

TEST_CASE("tri_energy")
{
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F1(1, 3);
    Eigen::MatrixXi F2(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F1 << 0, 1, 2;
    F2 << 0, 2, 1;
    double target_l = 0.5;
    triwild::TriWild m;
    m.create_mesh(V, F1);
    m.target_l = target_l;
    for (auto& t : m.get_faces()) {
        wmtk::logger().info(m.get_quality(t));
        wmtk::logger().info(m.get_quality(t) > 0);
        REQUIRE(m.get_quality(t) < 0);
    }
    triwild::TriWild m2;
    m2.create_mesh(V, F2);
    for (auto& t : m2.get_faces()) {
        wmtk::logger().info(m2.get_quality(t));
        wmtk::logger().info(m2.get_quality(t) > 0);
        REQUIRE(m2.get_quality(t) > 0);
    }
}

TEST_CASE("triwild_collapse", "[triwild_collapse][.]")
{
    // dummy case. Collapse 5 times. 1 tri
    const std::string root(WMT_DATA_DIR);
    const std::string path = "test_triwild_collapse_onboundary.obj";
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
    triwild::TriWild m;
    m.target_l = 1.;
    m.create_mesh(V, F, 0.01);
    for (auto& t : m.get_faces()) {
        assert(m.get_quality(t) > 0);
    }
    m.collapse_all_edges();
    m.consolidate_mesh();
    // REQUIRE(m.vertex_attrs[0].pos == Eigen::Vector2d(0., 5.));
    // REQUIRE(m.vertex_attrs[1].pos == Eigen::Vector2d(-5., 2.5));
    // REQUIRE(m.vertex_attrs[2].pos == Eigen::Vector2d(2.5, 1.875));
    m.write_obj("triwild_collapse.obj");
}

TEST_CASE("triwild_split", "[triwild_split][.]")
{
    // dummy case. swap 5 times. 1 tri
    const std::string root(WMT_DATA_DIR);
    const std::string path = "test_triwild.obj";
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
    // std::pair<Eigen::VectorXd, Eigen::VectorXd> box_minmax;
    // box_minmax = std::pair(V.colwise().minCoeff(), V.colwise().maxCoeff());
    // double diag = (box_minmax.first - box_minmax.second).norm();
    // double target_l = 0.5 * diag;
    // Load the mesh in the trimesh class
    triwild::TriWild m;
    m.target_l = 1.;
    m.create_mesh(V, F);
    m.split_all_edges();
    // m.consolidate_mesh();
    m.write_obj("triwild_split.obj");
    // REQUIRE(m.vertex_attrs[0].pos == Eigen::Vector2d(0., 5.));
    // REQUIRE(m.vertex_attrs[1].pos == Eigen::Vector2d(-5., 2.5));
    // REQUIRE(m.vertex_attrs[2].pos == Eigen::Vector2d(2.5, 1.875));
}

TEST_CASE("triwild_swap", "[triwild_swap][.]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = "test_triwild_swap.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);
    TriWild m;
    m.target_l = 5e-2;
    m.create_mesh(V, F, 0.01);
    for (auto& t : m.get_faces()) {
        REQUIRE(m.get_quality(t) > 0);
    }
    m.swap_all_edges();
    m.write_obj("triwild_swap.obj");
}
