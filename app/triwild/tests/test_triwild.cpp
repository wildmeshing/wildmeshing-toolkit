#include <TriWild.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/autodiff.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>

using namespace wmtk;
using namespace triwild;

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;

std::function<bool(std::array<double, 6>&)> is_inverted = [](auto& tri) {
    Eigen::Vector2d a, b, c;
    a << tri[0], tri[1];
    b << tri[2], tri[3];
    c << tri[4], tri[5];
    auto res = igl::predicates::orient2d(a, b, c);
    return (res != igl::predicates::Orientation::POSITIVE);
};
std::function<bool(std::array<double, 6>&)> is_degenerate = [](auto& tri) {
    Eigen::Vector3d a, b;
    a << tri[2] - tri[0], tri[3] - tri[1], 0.;
    b << tri[4] - tri[0], tri[5] - tri[1], 0.;
    auto area = (a.cross(b)).norm();
    auto long_e = std::max(a.norm(), b.norm());
    return (std::pow(area, 2) < 1e-5 || std::pow((area / long_e - 0.01), 2) < 1e-5);
};

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
    m2.set_energy(std::make_unique<wmtk::AMIPS>());

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
    m.set_energy(std::make_unique<wmtk::AMIPS>());

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
    m2.set_energy(std::make_unique<wmtk::AMIPS>());

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
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    m.split_all_edges();
    REQUIRE(m.vert_capacity() == 12);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
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
    m.set_energy(std::make_unique<wmtk::AMIPS>());

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
    m2.set_energy(std::make_unique<wmtk::AMIPS>());

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
    m.set_energy(std::make_unique<wmtk::AMIPS>());

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
    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    REQUIRE(E.rows() == 6);
    RowMatrix2<Scalar> V_aabb = V.block(0, 0, V.rows(), 2);
    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    REQUIRE(!aabb.empty());
    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
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
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    for (auto& t : m.get_faces()) {
        REQUIRE(m.get_quality(t) > 0);
    }
    // get the aabb tree for closest point detect in smooth projection
    RowMatrix2<Index> E = m.get_bnd_edge_matrix();

    RowMatrix2<Scalar> V_aabb = V.block(0, 0, V.rows(), 2);
    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };
    m.mesh_improvement(10);
    m.write_obj("triwild_improve_project.obj");

    m.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p;
    };
    m.mesh_improvement(10);
    m.write_obj("triwild_improve_wo_project.obj");
}

TEST_CASE("test_degenrate")
{
    std::array<double, 6> rand_tri = {27, 35, -14, -46, 26, 33};
    REQUIRE(is_degenerate(rand_tri));
}
TEST_CASE("symdi 2 rand tris")
{
    int pass = 0;
    for (int i = 0; i < 50; i++) {
        std::array<double, 6> rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        // // rand_tri = {0, 0, 2, 0, 1, 1.73};
        // rand_tri = {27, 35, -14, -46, 26, 33};
        if (is_degenerate(rand_tri)) {
            pass++;
            continue;
        }
        if (is_inverted(rand_tri)) {
            wmtk::logger().info("is_inverted");

            Eigen::Vector2d tmp;
            tmp << rand_tri[0], rand_tri[1];
            rand_tri[0] = rand_tri[2];
            rand_tri[1] = rand_tri[3];
            rand_tri[2] = tmp(0);
            rand_tri[3] = tmp(1);
        }
        wmtk::logger().info("target {}", rand_tri);
        std::function<double(const std::array<double, 6>&, int&)> SymDi_auto_value =
            [&rand_tri](auto& T, auto& i) {
                return wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getValue();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> SymDi_auto_grad =
            [&rand_tri](auto& T, auto& G, auto& i) {
                G = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getGradient();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)>
            SymDi_auto_hessian = [&rand_tri](auto& T, auto& H, auto& i) {
                H = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getHessian();
            };

        std::array<double, 6> rand_tri1 = rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri1[j] = rand() % 100 - 50;
        }
        if (is_degenerate(rand_tri1)) {
            pass++;
            continue;
        }

        if (is_inverted(rand_tri1)) {
            wmtk::logger().info("is_inverted");
            Eigen::Vector2d tmp;
            tmp << rand_tri1[0], rand_tri1[1];
            rand_tri1[0] = rand_tri1[2];
            rand_tri1[1] = rand_tri1[3];
            rand_tri1[2] = tmp(0);
            rand_tri1[3] = tmp(1);
        }
        wmtk::logger().info("input {}", rand_tri1);
        auto tri_output = wmtk::smooth_over_one_triangle(
            rand_tri1,
            SymDi_auto_value,
            SymDi_auto_grad,
            SymDi_auto_hessian);
        wmtk::logger().info("output {}", tri_output);
        wmtk::logger().info(
            "grad 1 {}",
            wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output).getGradient());
        wmtk::logger().info(
            "grad test2 {}",
            wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 1).getGradient());
        wmtk::logger().info(
            "grad test3 {}",
            wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 2).getGradient());
        if (std::pow(
                wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 0).getValue() - 4.0,
                2) < 1e-5)
            pass++;
    }
    wmtk::logger().info("number of successs {}", pass);
    REQUIRE(pass == 50);
}

TEST_CASE("amips 2 rand tris")
{
    int pass = 0;
    for (int i = 0; i < 50; i++) {
        std::array<double, 6> rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        if (is_degenerate(rand_tri)) {
            pass++;
            continue;
        }
        if (is_inverted(rand_tri)) {
            wmtk::logger().info("is_inverted");

            Eigen::Vector2d tmp;
            tmp << rand_tri[0], rand_tri[1];
            rand_tri[0] = rand_tri[2];
            rand_tri[1] = rand_tri[3];
            rand_tri[2] = tmp(0);
            rand_tri[3] = tmp(1);
        }

        wmtk::logger().info("target {}", rand_tri);
        std::function<double(const std::array<double, 6>&, int&)> AMIPS_auto_value =
            [&rand_tri](auto& T, auto& i) {
                return wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getValue();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> AMIPS_auto_grad =
            [&rand_tri](auto& T, auto& G, auto& i) {
                G = wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getGradient();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)>
            AMIPS_auto_hessian = [&rand_tri](auto& T, auto& H, auto& i) {
                H = wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getHessian();
            };

        std::array<double, 6> rand_tri1 = rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri1[j] = rand() % 100 - 50;
        }
        if (is_degenerate(rand_tri1)) {
            pass++;
            continue;
        }
        if (is_inverted(rand_tri1)) {
            wmtk::logger().info("is_inverted");
            Eigen::Vector2d tmp;
            tmp << rand_tri1[0], rand_tri1[1];
            rand_tri1[0] = rand_tri1[2];
            rand_tri1[1] = rand_tri1[3];
            rand_tri1[2] = tmp(0);
            rand_tri1[3] = tmp(1);
        }

        wmtk::logger().info("input {}", rand_tri1);
        auto tri_output = wmtk::smooth_over_one_triangle(
            rand_tri1,
            AMIPS_auto_value,
            AMIPS_auto_grad,
            AMIPS_auto_hessian);
        wmtk::logger().info("output {}", tri_output);
        wmtk::logger().info(
            "grad 1 {}",
            wmtk::AMIPS_autodiff_customize_target(rand_tri, tri_output).getGradient());
        wmtk::logger().info(
            "grad test2 {}",
            wmtk::AMIPS_autodiff_customize_target(rand_tri, tri_output, 1).getGradient());
        wmtk::logger().info(
            "grad test3 {}",
            wmtk::AMIPS_autodiff_customize_target(rand_tri, tri_output, 2).getGradient());
        if (std::pow(
                wmtk::AMIPS_autodiff_customize_target(rand_tri, tri_output, 0).getValue() - 2.0,
                2) < 1e-5)
            pass++;
    }
    wmtk::logger().info("number of successs {}", pass);
    REQUIRE(pass == 50);
}

TEST_CASE("symdi perturb one vert")
{
    int pass = 0;
    for (int i = 0; i < 50; i++) {
        std::array<double, 6> rand_tri;

        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        if (is_inverted(rand_tri)) {
            wmtk::logger().info("is_inverted");

            Eigen::Vector2d tmp;
            tmp << rand_tri[0], rand_tri[1];
            rand_tri[0] = rand_tri[2];
            rand_tri[1] = rand_tri[3];
            rand_tri[2] = tmp(0);
            rand_tri[3] = tmp(1);
        }
        if (is_degenerate(rand_tri)) {
            pass++;
            continue;
        }
        wmtk::logger().info("target {}", rand_tri);

        std::function<double(const std::array<double, 6>&, int&)> SymDi_auto_value =
            [&rand_tri](auto& T, auto& i) {
                return wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getValue();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> SymDi_auto_grad =
            [&rand_tri](auto& T, auto& G, auto& i) {
                G = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getGradient();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)>
            SymDi_auto_hessian = [&rand_tri](auto& T, auto& H, auto& i) {
                H = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getHessian();
            };

        std::array<double, 6> rand_tri1 = rand_tri;
        rand_tri1[0] += rand() % 20;
        rand_tri1[1] += rand() % 20;
        if (is_degenerate(rand_tri1)) {
            pass++;
            continue;
        }
        wmtk::logger().info("input {}", rand_tri1);

        auto tri_output = wmtk::smooth_over_one_triangle(
            rand_tri1,
            SymDi_auto_value,
            SymDi_auto_grad,
            SymDi_auto_hessian);
        wmtk::logger().info("output {}", tri_output);
        if (std::pow(
                wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 0).getValue() - 4.0,
                2) < 1e-5)
            pass++;

        else {
            wmtk::logger().info(
                "======fail with {}",
                wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 0).getValue());
        }
    }
    wmtk::logger().info("number of successs {}", pass);
    REQUIRE(pass == 50);
}


TEST_CASE("symdi same tri")
{
    int pass = 0;
    for (int i = 0; i < 100; i++) {
        std::array<double, 6> rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        if (is_inverted(rand_tri)) {
            wmtk::logger().info("is_inverted");

            Eigen::Vector2d tmp;
            tmp << rand_tri[0], rand_tri[1];
            rand_tri[0] = rand_tri[2];
            rand_tri[1] = rand_tri[3];
            rand_tri[2] = tmp(0);
            rand_tri[3] = tmp(1);
        }
        if (is_degenerate(rand_tri)) {
            pass++;
            continue;
        }
        wmtk::logger().info("target {}", rand_tri);

        std::function<double(const std::array<double, 6>&, int&)> SymDi_auto_value =
            [&rand_tri](auto& T, auto& i) {
                return wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getValue();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> SymDi_auto_grad =
            [&rand_tri](auto& T, auto& G, auto& i) {
                G = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getGradient();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)>
            SymDi_auto_hessian = [&rand_tri](auto& T, auto& H, auto& i) {
                H = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getHessian();
            };

        std::array<double, 6> rand_tri1 = rand_tri;
        auto tri_output = wmtk::smooth_over_one_triangle(
            rand_tri1,
            SymDi_auto_value,
            SymDi_auto_grad,
            SymDi_auto_hessian);
        if (std::pow(
                wmtk::SymDi_autodiff_customize_target(rand_tri, tri_output, 0).getValue() - 4.0,
                2) < 1e-5)
            pass++;
    }
    wmtk::logger().info("number of successs {}", pass);

    REQUIRE(pass == 100);
}

TEST_CASE("amips same tri")
{
    int pass = 0;
    for (int i = 0; i < 100; i++) {
        std::array<double, 6> rand_tri;
        for (int j = 0; j < 6; j++) {
            rand_tri[j] = rand() % 100 - 50;
        }
        if (is_inverted(rand_tri)) {
            wmtk::logger().info("is_inverted");

            Eigen::Vector2d tmp;
            tmp << rand_tri[0], rand_tri[1];
            rand_tri[0] = rand_tri[2];
            rand_tri[1] = rand_tri[3];
            rand_tri[2] = tmp(0);
            rand_tri[3] = tmp(1);
        }
        if (is_degenerate(rand_tri)) {
            pass++;
            continue;
        }
        std::function<double(const std::array<double, 6>&, int&)> AMIPS_auto_value =
            [&rand_tri](auto& T, auto& i) {
                return wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getValue();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> AMIPS_auto_grad =
            [&rand_tri](auto& T, auto& G, auto& i) {
                G = wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getGradient();
            };
        std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)>
            AMIPS_auto_hessian = [&rand_tri](auto& T, auto& H, auto& i) {
                H = wmtk::AMIPS_autodiff_customize_target(rand_tri, T, i).getHessian();
            };

        std::array<double, 6> rand_tri1 = rand_tri;
        auto tri_output = wmtk::smooth_over_one_triangle(
            rand_tri1,
            AMIPS_auto_value,
            AMIPS_auto_grad,
            AMIPS_auto_hessian);
        if (std::pow(
                wmtk::AMIPS_autodiff_customize_target(rand_tri, tri_output, 0).getValue() - 2.0,
                2) < 1e-5)
            pass++;
    }
    REQUIRE(pass == 100);
}

TEST_CASE("symdi rototranslation energy")
{
    // given 2 triangles only differ by rotation
    std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
    std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
    REQUIRE(
        std::pow((SymDi_autodiff_customize_target(rand_tri, rand_tri1).getValue() - 4.0), 2) <
        1e-5);
}

TEST_CASE("amips rototranslation energy")
{
    // given 2 triangles only differ by rotation
    std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
    std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
    REQUIRE(
        std::pow((AMIPS_autodiff_customize_target(rand_tri, rand_tri1).getValue() - 2.0), 2) <
        1e-5);
}

TEST_CASE("symdi with energy scaling")
{
    // input
    std::array<double, 6> rand_tri = {0., 0., 5, -1., -3., 10.};
    std::array<double, 6> rand_tri1 = rand_tri;
    // using input to masage a target
    Eigen::Vector3d ac;
    ac << rand_tri[4] - rand_tri[0], rand_tri[5] - rand_tri[1], 0.0;
    Eigen::Vector3d ab;
    ab << rand_tri[2] - rand_tri[0], rand_tri[3] - rand_tri[1], 0.0;
    double S = ((ac.cross(ab)).norm()) / 2.;
    double r = sqrt(S);
    assert(r > 0);
    // 0, 0, 2* 1/sqrt(sqrt(3)),0, 1/sqrt(sqrt(3)), sqrt(sqrt(3))
    rand_tri = {0, 0, r * 2 * 1 / sqrt(sqrt(3)), 0, r * 1 / sqrt(sqrt(3)), r * sqrt(sqrt(3))};
    std::function<double(const std::array<double, 6>&, int&)> SymDi_auto_value =
        [&rand_tri](auto& T, auto& i) {
            return wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getValue();
        };
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> SymDi_auto_grad =
        [&rand_tri](auto& T, auto& G, auto& i) {
            G = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getGradient();
        };
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)> SymDi_auto_hessian =
        [&rand_tri](auto& T, auto& H, auto& i) {
            H = wmtk::SymDi_autodiff_customize_target(rand_tri, T, i).getHessian();
        };

    auto tri_output = wmtk::smooth_over_one_triangle(
        rand_tri1,
        SymDi_auto_value,
        SymDi_auto_grad,
        SymDi_auto_hessian);
    wmtk::logger().info("output is {}", tri_output);

    Eigen::Vector2d a, b, c;
    a << tri_output[0], tri_output[1];
    b << tri_output[2], tri_output[3];
    c << tri_output[4], tri_output[5];
    // check it is equilateral
    REQUIRE(std::pow((b - a).norm() - (c - a).norm(), 2) < 1e-5);
    REQUIRE(std::pow((b - c).norm() - (a - c).norm(), 2) < 1e-5);
    // check the area is same
    ac << tri_output[4] - tri_output[0], tri_output[5] - tri_output[1], 0.0;

    ab << tri_output[2] - tri_output[0], tri_output[3] - tri_output[1], 0.0;
    auto area = ((ac.cross(ab)).norm()) / 2.;

    REQUIRE(std::pow(area - S, 2) < 1e-5);
}

TEST_CASE("smoothing_symdi_scaling")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F, -1, false);

    m.set_energy(std::make_unique<wmtk::SymDi>());
    m.m_target_l = 2;
    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    assert(m.check_mesh_connectivity_validity());
    m.smooth_all_vertices();
    std::array<double, 6> T;
    double r = 2;
    std::array<double, 6> target_tri = {0, 0, r * 1, 0, r * 1 / 2, r * sqrt(3) / 2};

    for (auto i = 0; i < 3; ++i) {
        T[i * 2] = m.vertex_attrs[i].pos[0];
        T[i * 2 + 1] = m.vertex_attrs[i].pos[1];
    }
    // energy is 4.0 after smooth
    REQUIRE(abs(wmtk::SymDi_autodiff_customize_target(target_tri, T, 0).getValue() - 4) < 1e-3);
    REQUIRE(abs(wmtk::SymDi_autodiff_customize_target(target_tri, T, 1).getValue() - 4) < 1e-3);
    REQUIRE(abs(wmtk::SymDi_autodiff_customize_target(target_tri, T, 2).getValue() - 4) < 1e-3);

    // grad is (0,0) after smooth
    REQUIRE((wmtk::SymDi_autodiff_customize_target(target_tri, T, 0).getGradient()).norm() < 1e-2);
    REQUIRE((wmtk::SymDi_autodiff_customize_target(target_tri, T, 1).getGradient()).norm() < 1e-2);
    REQUIRE((wmtk::SymDi_autodiff_customize_target(target_tri, T, 2).getGradient()).norm() < 1e-2);
    m.write_obj("smoothing_symdi_test.obj");
}

TEST_CASE("smoothing_amips_scaling")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    assert(m.check_mesh_connectivity_validity());
    m.set_energy(std::make_unique<wmtk::AMIPS>());
    m.m_target_l = 2;
    m.smooth_all_vertices();
    std::array<double, 6> T;
    double r = 2;
    std::array<double, 6> target_tri = {0, 0, r * 1, 0, r * 1 / 2, r * sqrt(3) / 2};

    for (auto i = 0; i < 3; ++i) {
        T[i * 2] = m.vertex_attrs[i].pos[0];
        T[i * 2 + 1] = m.vertex_attrs[i].pos[1];
    }

    // energy is 2.0 after smooth
    REQUIRE(abs(wmtk::AMIPS_autodiff_customize_target(target_tri, T, 0).getValue() - 2) < 1e-3);
    REQUIRE(abs(wmtk::AMIPS_autodiff_customize_target(target_tri, T, 1).getValue() - 2) < 1e-3);
    REQUIRE(abs(wmtk::AMIPS_autodiff_customize_target(target_tri, T, 2).getValue() - 2) < 1e-3);

    // grad is (0,0) after smooth
    REQUIRE((wmtk::AMIPS_autodiff_customize_target(target_tri, T, 0).getGradient()).norm() < 1e-3);
    REQUIRE((wmtk::AMIPS_autodiff_customize_target(target_tri, T, 1).getGradient()).norm() < 1e-3);
    REQUIRE((wmtk::AMIPS_autodiff_customize_target(target_tri, T, 2).getGradient()).norm() < 1e-3);
    m.write_obj("smoothing_amips_test.obj");
}

TEST_CASE("remeshing_symdi_scaling")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F, -1, false);
    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    m.set_projection(aabb);
    m.set_energy(std::make_unique<wmtk::SymDi>());
    m.m_target_l = 2;

    assert(m.check_mesh_connectivity_validity());
    m.mesh_improvement(10);
    m.write_obj("remeshing_symdi_yesboundary.obj");
}

TEST_CASE("edge_length_energy_one_triangle_constant")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 2;
    // m.split_all_edges();

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_constant_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_one_triangle_linear")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(u); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 2;

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_one_triangle_dramatic_linear")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 2;

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_one_triangle_constant_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    m.set_projection(aabb);
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 0.5;
    m.mesh_improvement(3);
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_constant_remesh_yesboundary.obj",
        displacement_double);
}
TEST_CASE("edge_length_energy_one_triangle_linear_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(u); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    m.set_projection(aabb);
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 0.5;
    m.mesh_improvement(10);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_linear_remesh_yesboundary.obj",
        displacement_double);
}
TEST_CASE("edge_length_energy_one_triangle_dramatic_linear_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // bool ok = igl::read_triangle_mesh("after_split_0.obj", V, F);
    // assert(ok);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_triwild_displacement = displacement_vector;

    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);
    m.set_projection(aabb);
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.m_target_l = 1;
    m.mesh_improvement(15);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_remesh_yesboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_smooth_verify")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("split_perfect_mesh.obj", V, F);
    TriWild m;
    assert(ok);
    m.create_mesh(V, F, -1, false);
    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);

    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };

    m.set_projection(aabb);
    assert(m.invariants(m.get_faces()));

    m.m_target_l = 1;
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.smooth_all_vertices();
    for (auto f : m.get_faces()) assert(!m.is_inverted(f));
    assert(m.invariants(m.get_faces()));
    m.write_displaced_obj("same_perfect_mesh_0.021x10.obj", displacement_double);
}

TEST_CASE("edge_length_energy_collapse_verify")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("same_perfect_mesh_0.021.obj", V, F);
    TriWild m;
    assert(ok);
    m.create_mesh(V, F, -1, false);
    RowMatrix2<Index> E = m.get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(m.vert_capacity(), 2);
    for (int i = 0; i < m.vert_capacity(); ++i) {
        V_aabb.row(i) << m.vertex_attrs[i].pos[0], m.vertex_attrs[i].pos[1];
    }

    lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb(V_aabb, E);

    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };

    m.set_projection(aabb);
    assert(m.invariants(m.get_faces()));

    m.m_target_l = 1;
    m.set_energy(std::make_unique<wmtk::EdgeLengthEnergy>(displacement_vector));
    m.collapse_all_edges();
    for (auto f : m.get_faces()) assert(!m.is_inverted(f));
    assert(m.invariants(m.get_faces()));
    m.write_displaced_obj("collapse_perfect_mesh_0.021.obj", displacement_double);
}

TEST_CASE("boundary parametrization")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    wmtk::Boundary bnd;
    bnd.construct_boudaries(V, F);

    REQUIRE(bnd.m_boundaries.size() == 1);
    REQUIRE(bnd.m_arclengths.size() == 1);

    REQUIRE(bnd.m_arclengths[0].size() == 4);

    REQUIRE(bnd.m_arclengths[0][0] == 0);
    REQUIRE(bnd.m_arclengths[0][1] == 10);
    REQUIRE(bnd.m_arclengths[0][2] == 10 + 10 * sqrt(2));
    REQUIRE(bnd.m_arclengths[0][3] == 10 + 10 + 10 * sqrt(2));

    Eigen::Vector2d test_v0(0, 0);
    auto t0 = bnd.uv_to_t(test_v0);
    REQUIRE(t0 == 0.);
    auto v0 = bnd.t_to_uv(0, t0);
    REQUIRE(v0 == test_v0);

    Eigen::Vector2d test_v1(5, 0);
    auto t1 = bnd.uv_to_t(test_v1);
    REQUIRE(t1 == 5.);
    auto v1 = bnd.t_to_uv(0, t1);
    REQUIRE(v1 == test_v1);

    Eigen::Vector2d test_v2(5, 5);
    auto t2 = bnd.uv_to_t(test_v2);
    REQUIRE(t2 == 10 + 5. * sqrt(2));
    auto v2 = bnd.t_to_uv(0, t2);
    REQUIRE(v2 == test_v2);

    Eigen::Vector2d test_v3(0, 0.1);
    auto t3 = bnd.uv_to_t(test_v3);
    REQUIRE(t3 == 10 + 10. * sqrt(2) + 9.9);
    auto v3 = bnd.t_to_uv(0, t3);
    REQUIRE((v3 - test_v3).stableNorm() < 1e-8);
}

TEST_CASE("new boundary")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F, -1, false);
    m.m_get_closest_point = [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d { return p; };
    m.m_target_l = 4;
    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
}