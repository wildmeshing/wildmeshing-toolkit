#include <TriWild.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>

#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>

#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>


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
    const std::string path = root + "/test_triwild_collapse_onboundary.obj";
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

    // without envelop. boundary is locked, nothing changes
    // center vertex have 7 tris
    triwild::TriWild m;
    m.m_target_l = 1.;
    m.create_mesh(V, F, -1, true);
    for (auto& t : m.get_faces()) {
        assert(m.get_quality(t) > 0);
    }
    m.collapse_all_edges();
    m.consolidate_mesh();
    for (auto v : m.get_vertices()) {
        if (v.vid(m) == 2) REQUIRE(m.get_valence_for_vertex(v) == 7);
    }
    m.write_obj("triwild_collapse_freeze.obj");

    // with envelop. boundary allowed to move in envelop
    // center vertex have 7 tris
    triwild::TriWild m2;
    m2.m_target_l = 1.;
    m2.create_mesh(V, F, 0.01);
    for (auto& t : m2.get_faces()) {
        assert(m2.get_quality(t) > 0);
    }
    m2.collapse_all_edges();
    m2.consolidate_mesh();
    for (auto v : m2.get_vertices()) {
        if (v.vid(m2) == 2) REQUIRE(m2.get_valence_for_vertex(v) == 6);
    }
    m2.write_obj("triwild_collapse_envelop.obj");
}

TEST_CASE("triwild_split", "[triwild_split][.]")
{
    // dummy case. swap 5 times. 1 tri
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
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

    // edges are split regardless of envelope or not
    triwild::TriWild m;
    m.m_target_l = 1.;
    m.create_mesh(V, F);
    m.split_all_edges();
    REQUIRE(m.vert_capacity() == 12);
    m.write_obj("triwild_split.obj");
}

TEST_CASE("triwild_swap", "[triwild_swap][.]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild_swap.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    // without envelop. boundary is locked, nothing changes
    // center vertex have 7 tris
    TriWild m;
    m.m_target_l = 5e-2;
    m.create_mesh(V, F, -1, true);
    for (auto& t : m.get_faces()) {
        REQUIRE(m.get_quality(t) > 0);
    }
    m.swap_all_edges();
    for (auto v : m.get_vertices()) {
        if (v.vid(m) == 2) REQUIRE(m.get_valence_for_vertex(v) == 7);
    }
    m.write_obj("triwild_swap_freeze.obj");
    // with envelop. can be swapped
    // center vertex have 6 tris after swap
    TriWild m2;
    m2.m_target_l = 5e-2;
    m2.create_mesh(V, F, 0.01);
    for (auto& t : m2.get_faces()) {
        REQUIRE(m2.get_quality(t) > 0);
    }
    m2.swap_all_edges();
    for (auto v : m2.get_vertices()) {
        if (v.vid(m2) == 2) REQUIRE(m2.get_valence_for_vertex(v) == 6);
    }
    m.write_obj("triwild_swap_envelop.obj");
}

TEST_CASE("triwild_improve")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    REQUIRE(ok);
    TriWild m;
    m.m_target_l = 0.5;
    m.m_stop_energy = 2.0;
    m.create_mesh(V, F, -1, true);
    m.mesh_improvement(10);
    m.write_obj("triwild_improve_freezebnd.obj");
}

TEST_CASE("autodiff")
{
    for (int i = 0; i < 100; i++) {
        std::array<double, 6> rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        REQUIRE(std::pow(AMIPS2D_energy(rand_tri) - AMIPS_autodiff(rand_tri).getValue(), 2) < 1e-4);
        Eigen::Vector2d Jac;
        AMIPS2D_jacobian(rand_tri, Jac);
        REQUIRE((Jac - AMIPS_autodiff(rand_tri).getGradient()).norm() < 1e-4);
        Eigen::Matrix2d Hes;
        AMIPS2D_hessian(rand_tri, Hes);
        REQUIRE((Hes - AMIPS_autodiff(rand_tri).getHessian()).norm() < 1e-4);
    }
}
TEST_CASE("AABB")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);
    TriWild m;
    m.create_mesh(V, F, 0.2, false);
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> E = m.get_bnd_edge_matrix();
    REQUIRE(E.rows() == 6);
    Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> V_aabb = V.block(0, 0, V.rows(), 2);
    lagrange::bvh::EdgeAABBTree<
        Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>,
        Eigen::Matrix<unsigned long, Eigen::Dynamic, 2, Eigen::RowMajor>,
        2>
        aabb(V_aabb, E);
    REQUIRE(!aabb.empty());
    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        unsigned long ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };
    auto result = m.m_get_closest_point(Eigen::RowVector2d(-0.7, 0.6));
    REQUIRE(result == Eigen::RowVector2d(-1, 0.6));
}

TEST_CASE("improve with AABB")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    // without envelop. boundary is locked, nothing changes
    // center vertex have 7 tris
    TriWild m;
    m.m_target_l = 5e-2;
    m.create_mesh(V, F, 0.2, false);
    for (auto& t : m.get_faces()) {
        REQUIRE(m.get_quality(t) > 0);
    }
    // get the aabb tree for closest point detect in smooth projection
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> E = m.get_bnd_edge_matrix();

    Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> V_aabb = V.block(0, 0, V.rows(), 2);
    lagrange::bvh::EdgeAABBTree<
        Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>,
        Eigen::Matrix<unsigned long, Eigen::Dynamic, 2, Eigen::RowMajor>,
        2>
        aabb(V_aabb, E);
    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        unsigned long ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };
    m.mesh_improvement(10);
    m.write_obj("triwild_improve_project.obj");

    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        unsigned long ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p;
    };
    m.mesh_improvement(10);
    m.write_obj("triwild_improve_wo_project.obj");
}