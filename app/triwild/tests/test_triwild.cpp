#include <TriWild.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/AdaptiveGuassQuadrature.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <functional>
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
    m.mesh_parameters.m_target_l = 1.;
    m.create_mesh(V, F);
    m.mesh_parameters.m_bnd_freeze = true;
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
    m.mesh_parameters.m_target_l = 1.;
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
    m.mesh_parameters.m_target_l = 5e-2;
    m.create_mesh(V, F);
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
    m2.mesh_parameters.m_target_l = 5e-2;
    m2.create_mesh(V, F);
    m2.mesh_parameters.m_bnd_freeze = true;
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
    m.mesh_parameters.m_target_l = 0.5;
    m.mesh_parameters.m_stop_energy = 2.0;
    m.create_mesh(V, F);
    m.mesh_parameters.m_bnd_freeze = true;
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
    m.create_mesh(V, F);
    m.set_projection();

    auto result = m.mesh_parameters.m_get_closest_point(Eigen::RowVector2d(-0.7, 0.6));
    REQUIRE(result == Eigen::RowVector2d(-1, 0.6));
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
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_energy(std::make_unique<wmtk::SymDi>());
    assert(m.mesh_parameters.m_energy != nullptr);
    m.mesh_parameters.m_target_l = 2;
    m.mesh_parameters.m_boundary_parameter = false;
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
    m.create_mesh(V, F);
    assert(m.check_mesh_connectivity_validity());
    m.set_energy(std::make_unique<wmtk::AMIPS>());
    m.mesh_parameters.m_target_l = 2;
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
    m.create_mesh(V, F);
    m.set_projection();
    m.set_energy(std::make_unique<wmtk::SymDi>());
    m.mesh_parameters.m_target_l = 2;

    assert(m.check_mesh_connectivity_validity());
    m.mesh_improvement(10);
    m.write_obj("remeshing_symdi_yesboundary.obj");
}

TEST_CASE("edge_length_energy_smooth_constant")
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
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, ENERGY_TYPE::EDGE_LENGTH, false);

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_constant_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_smooth_linear")
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
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, ENERGY_TYPE::EDGE_LENGTH, false);

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_smooth_dramatic_linear")
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
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, ENERGY_TYPE::EDGE_LENGTH, false);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_constant_remesh")
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
    m.create_mesh(V, F);
    m.set_parameters(0.5, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
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
    m.create_mesh(V, F);
    m.set_parameters(0.5, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    m.mesh_improvement(3);

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
    // create the json file to record logs
    std::ofstream js_o("dramatic_linear_nobnd.json");
    m.create_mesh(V, F);
    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, false);
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.mesh_improvement(100);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_remesh_noboundary.obj",
        displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
}
TEST_CASE("edge_length_energy_one_triangle_smooth_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return sin(u * M_PI) + cos(v * M_PI);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u) / 10., DScalar(v) / 10.).getValue());
        return p;
    };
    TriWild m;
    // create the json file to record logs
    std::ofstream js_o("smooth_yesbnd.json");
    m.create_mesh(V, F);
    m.set_parameters(0.1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].fixed);
    }
    m.mesh_improvement(3);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_smooth_remesh_yesoboundary.obj",
        displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
}

TEST_CASE("smoothing_gradient_debug")
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
    // create the json file to record logs
    std::ofstream js_o("gradient_debug_yesbnd.json");
    m.create_mesh(V, F);
    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    for (auto v : m.get_vertices()) {
        // m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.gradient_debug(1000);

    m.write_displaced_obj("smooth_gradient_debug_yesbnd.obj", displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
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

    double t;

    Eigen::Vector2d test_v0(0, 0);
    auto t0 = bnd.uv_to_t(test_v0);
    REQUIRE(t0 == 0.);
    auto v0 = bnd.t_to_uv(0, t0);
    REQUIRE(v0 == test_v0);
    auto ij0 = bnd.uv_to_ij(v0, t);
    assert(t == t0);
    REQUIRE(ij0.first == 0);
    REQUIRE(ij0.second == 0);

    Eigen::Vector2d test_v1(5, 0);
    auto t1 = bnd.uv_to_t(test_v1);
    REQUIRE(t1 == 5.);
    auto v1 = bnd.t_to_uv(0, t1);
    REQUIRE(v1 == test_v1);
    auto ij1 = bnd.uv_to_ij(v1, t);
    REQUIRE(t == t1);
    REQUIRE(ij1.first == 0);
    REQUIRE(ij1.second == 0);

    Eigen::Vector2d test_v2(5, 5);
    auto t2 = bnd.uv_to_t(test_v2);
    REQUIRE(t2 == 10 + 5. * sqrt(2));
    auto v2 = bnd.t_to_uv(0, t2);
    REQUIRE(v2 == test_v2);
    auto ij2 = bnd.uv_to_ij(v2, t);
    REQUIRE(t == t2);
    REQUIRE(ij2.first == 0);
    REQUIRE(ij2.second == 1);

    Eigen::Vector2d test_v3(0, 0.1);
    auto t3 = bnd.uv_to_t(test_v3);
    REQUIRE(t3 == 10 + 10. * sqrt(2) + 9.9);
    auto v3 = bnd.t_to_uv(0, t3);
    REQUIRE((v3 - test_v3).stableNorm() < 1e-8);
    auto ij3 = bnd.uv_to_ij(v3, t);
    REQUIRE(t == t3);
    REQUIRE(ij3.first == 0);
    REQUIRE(ij3.second == 2);
}

TEST_CASE("boundary parameter smooth")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F);
    m.set_projection();

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        (void)u;
        (void)v;
        return DScalar(1);
    };
    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };
    m.set_parameters(4, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    m.smooth_all_vertices();
    m.write_displaced_obj("smooth_new_boundary.obj", displacement_double);

    for (auto v : m.get_vertices()) {
        auto v_project = m.mesh_parameters.m_get_closest_point(m.vertex_attrs[v.vid(m)].pos);
        REQUIRE((v_project.transpose() - m.vertex_attrs[v.vid(m)].pos).squaredNorm() < 1e-5);
    }
}

TEST_CASE("boundary parameter split")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F);
    m.set_projection();
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };

    m.set_parameters(4, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    for (auto f : m.get_faces()) REQUIRE(!m.is_inverted(f));
    m.split_all_edges();
    m.write_displaced_obj("split_new_boundary.obj", displacement_double);

    for (auto e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
            auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                0,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose()).squaredNorm() <
                1e-5);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose()).squaredNorm() <
                1e-5);
        }
    }
}

TEST_CASE("boundary parameter collapse")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("../build_release/after_split_7.obj", V, F);
    assert(ok);

    TriWild m;
    m.create_mesh(V, F);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [](double u, double v) -> double { return 10 * u; };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };

    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    m.collapse_all_edges();
    m.consolidate_mesh();
    m.write_displaced_obj("collapse_new_boundary.obj", displacement_double);

    for (auto e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
            auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                0,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose()).squaredNorm() <
                1e-5);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose()).squaredNorm() <
                1e-5);
        }
    }
}

TEST_CASE("energy gradient")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_remesh_yesboundary.obj",
        V,
        F);
    assert(ok);

    TriWild m;
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };
    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    Eigen::VectorXd v_flat;
    m.flatten_dofs(v_flat);
    Eigen::VectorXd finitediff_grad = Eigen::VectorXd::Zero(m.get_vertices().size() * 2);

    std::function<double(const Eigen::VectorXd&)> f =
        [&m](const Eigen::VectorXd& v_flat) -> double { return m.get_mesh_energy(v_flat); };
    fd::finite_gradient(v_flat, f, finitediff_grad, fd::SECOND, 1e-2);

    for (auto v : m.get_vertices()) {
        auto autodiff = m.get_one_ring_energy(v);
        wmtk::logger().info(
            "boundary {} autodiff {} finitediff {}, {}",
            m.is_boundary_vertex(v),
            autodiff.second,
            finitediff_grad[v.vid(m) * 2],
            finitediff_grad[v.vid(m) * 2 + 1]);
    }
}

TEST_CASE("gradient")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("split_new_boundary.obj", V, F);
    assert(ok);
    TriWild m;
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };
    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    Eigen::VectorXd v_flat, finitediff_grad;
    m.flatten_dofs(v_flat);
    finitediff_grad.resize(v_flat.size());
    std::function<double(const Eigen::VectorXd&)> f =
        [&m](const Eigen::VectorXd& v_flat) -> double {
        double energy = 0;
        for (int i = 0; i < v_flat.size(); i++) {
            if (v_flat(i) == std::numeric_limits<double>::infinity()) continue;
            wmtk::logger().info("v_flat({}) {}", i, v_flat(i));
            energy += v_flat(i);
        }
        return energy;
    };
    fd::finite_gradient(v_flat, f, finitediff_grad, fd::SECOND, 1e-2);
}

TEST_CASE("line_parametrization")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    DiffScalarBase::setVariableCount(2);

    DScalar t(0, 0);
    Eigen::Vector2d A(10, 0);
    Eigen::Vector2d B(7.5, 2.5);
    Eigen::Matrix<DScalar, 2, 1> ad_n;
    ad_n << t * (B - A).normalized()(0), t * (B - A).normalized()(1);
    wmtk::logger().info("ad_n {}", ad_n);
    wmtk::logger().info(
        "but they are actually {} {}",
        t * (B - A).normalized()(0),
        t * (B - A).normalized()(1));
    auto x = A(0) + t * (B - A).normalized()(0);
    auto y = A(1) + t * (B - A).normalized()(1);
    wmtk::logger().info("tmpA_x {} ", x);
    wmtk::logger().info("tmpA_y {}", y);
    auto f = pow(5 - x, 2) + pow(0 - y, 2);
    f = pow(f - 1, 2);
    wmtk::logger().info(f.getValue());
    wmtk::logger().info(f.getGradient());
    wmtk::logger().info(f);

    Eigen::VectorXd fd_t, finitediff_grad;
    finitediff_grad.resize(1);
    fd_t.resize(1);
    fd_t(0) = 0.;
    std::function<double(const Eigen::VectorXd&)> fd_f =
        [&A, &B](const Eigen::VectorXd& fd_t) -> double {
        double energy = 0;
        auto n = (B - A).normalized();
        wmtk::logger()
            .info("fd_t {} A {} B {}", fd_t(0), (A(0) + fd_t(0) * n(0)), (A(1) + fd_t(0) * n(1)));
        energy += pow(5 - (A(0) + fd_t(0) * n(0)), 2) + pow(0 - (A(1) + fd_t(0) * n(1)), 2);
        energy = pow(energy - 1, 2);
        wmtk::logger().info("energy {}", energy);
        return energy;
    };
    fd::finite_gradient(fd_t, fd_f, finitediff_grad, fd::SECOND, 1e-2);
}

TEST_CASE("exr saving and loading")
{
    Image image(10, 10);
    auto displacement_double = [](const double& u, const double& v) -> double { return 10 * u; };
    image.set(displacement_double);
    image.save("tryout.exr");
    Image image2(10, 10);
    image2.load("tryout.exr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    for (int i = 0; i < 10; i++) {
        auto p = static_cast<float>(0.1 * i);
        wmtk::logger().info(
            "at p = {},{} , image2 {}, image {}",
            0.1 * i,
            0.1 * i,
            image2.get(p, p),
            image.get(p, p));
        REQUIRE(abs(image2.get(p, p) - image.get(p, p)) < 1e-4);
    }
    // test bicubic interpolation
    Image image3(512, 512);
    image3.load(
        "/home/yunfan/data/plastic_stripes_Height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    std::mt19937 rand_generator;
    std::uniform_real_distribution<double> rand_dist;
    for (int j = 0; j < 10; j++) {
        Eigen::Vector2d rand_p;
        rand_p = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));

        wmtk::logger().info(
            "image3 at {} : {} ",
            rand_p,
            image3.get(static_cast<float>(rand_p(0)), static_cast<float>(rand_p(1))));
    }
}

TEST_CASE("hdr saving and loading")
{
    Image image(512, 512);
    image.load(
        "/home/yunfan/data/one_ramp.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    // auto displacement_stripe = [&image](const DScalar& u, const DScalar& v) -> DScalar {
    //     return image.get(u, v);
    // };
    auto displacement_double = [&image](const double& u, const double& v) -> float {
        auto x = static_cast<float>(u);
        auto y = static_cast<float>(v);
        return image.get(x, y);
    };
    Image image2(1024, 1024);
    image2.set(displacement_double);
    image2.save("interpolated_one_ramp.hdr");

    Image image3(1024, 1024);
    image3.load(
        "interpolated_one_ramp.hdr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    std::mt19937 rand_generator;
    std::uniform_real_distribution<double> rand_dist;
    for (int j = 0; j < 10; j++) {
        Eigen::Vector2d rand_p;
        rand_p = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));

        wmtk::logger().info(
            "image2 at {} : {} =? {}",
            rand_p,
            image3.get(static_cast<float>(rand_p.x()), static_cast<float>(rand_p.y())),
            displacement_double(rand_p.x(), rand_p.y()));
    }
}

TEST_CASE("bicubic interpolation")
{
    // Create a random bivariate cubic polynomial
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Matrix4f M = Eigen::Matrix4f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto bivariate_cubic_polynomial = [&](double x, double y) -> double {
        Eigen::Vector4f X(1, x, x * x, x * x * x);
        Eigen::Vector4f Y(1, y, y * y, y * y * y);
        return X.transpose() * M * Y;
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(bivariate_cubic_polynomial);

    // Sample random points in the image and check correctness.
    // Avoid image boundaries since finite diff will be off at image border.
    std::uniform_real_distribution<float> dist_samples(0.1, 0.9);

    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        auto value0 = image.get(p.x(), p.y());
        auto expected = bivariate_cubic_polynomial(p.x(), p.y());
        CAPTURE(k, p.x(), p.y(), value0, expected);
        REQUIRE_THAT(value0, Catch::Matchers::WithinRel(expected, 0.001));
    }
}

TEST_CASE("bicubic autodiff")
{
    // Create a random bivariate cubic polynomial
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Matrix4f M = Eigen::Matrix4f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto bivariate_cubic_polynomial = [&](auto x, auto y) {
        using T = std::decay_t<decltype(x)>;
        Eigen::Vector4<T> X(T(1), x, x * x, x * x * x);
        Eigen::Vector4<T> Y(T(1), y, y * y, y * y * y);
        return T(X.transpose() * M.cast<T>() * Y);
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(bivariate_cubic_polynomial);

    // Sample random points in the image and check correctness.
    std::uniform_real_distribution<float> dist_samples(0.1, 0.9);

    DiffScalarBase::setVariableCount(2);
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        DScalar x(0, p.x());
        DScalar y(1, p.y());
        DScalar value0 = image.get(x, y);
        DScalar expected = bivariate_cubic_polynomial(x, y);
        auto g0 = value0.getGradient().eval();
        auto g1 = expected.getGradient().eval();
        auto h0 = value0.getHessian().eval();
        auto h1 = expected.getHessian().eval();

        auto i = static_cast<int>(std::floor(p.x() * width - Scalar(0.5)));
        auto j = static_cast<int>(std::floor(p.y() * height - Scalar(0.5)));
        CAPTURE(k, i, j, p.x(), p.y(), value0, expected);
        REQUIRE_THAT(value0.getValue(), Catch::Matchers::WithinRel(expected.getValue(), 1e-2));
        for (int k = 0; k < g0.size(); ++k) {
            REQUIRE_THAT(g0[k], Catch::Matchers::WithinRel(g1[k], 1e-3));
        }
        for (int k = 0; k < h0.size(); ++k) {
            REQUIRE_THAT(
                h0.data()[k],
                Catch::Matchers::WithinRel(h1.data()[k], 1e-3) ||
                    Catch::Matchers::WithinAbs(0, 1e-3));
        }
    }
}

TEST_CASE("bicubic periodic")
{
    // Use periodic analytical function
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Vector2f coeffs = Eigen::Vector2f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto periodic_function = [&](auto x, auto y) {
        using T = std::decay_t<decltype(x)>;
        T z = coeffs[0] * sin(x * 2.f * M_PI) + coeffs[1] * sin(y * 2.f * M_PI);
        return z;
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(periodic_function, WrappingMode::REPEAT, WrappingMode::REPEAT);

    // Sample random points in the image and check correctness.
    std::uniform_real_distribution<float> dist_samples(0, 1);

    DiffScalarBase::setVariableCount(2);
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        DScalar x(0, p.x());
        DScalar y(1, p.y());
        DScalar value0 = image.get(x, y);
        DScalar expected = periodic_function(x, y);
        auto g0 = value0.getGradient().eval();
        auto g1 = expected.getGradient().eval();
        auto h0 = value0.getHessian().eval();
        auto h1 = expected.getHessian().eval();

        auto i = static_cast<int>(std::floor(p.x() * width - Scalar(0.5)));
        auto j = static_cast<int>(std::floor(p.y() * height - Scalar(0.5)));
        CAPTURE(k, i, j, p.x(), p.y(), value0, expected);

        // Since the function is not truly a cubic polynomial, we relax our gradient/hessian
        // tolerance to a much lower value... but hopefully this is enough to check that our signal
        // is periodic over the image.
        REQUIRE_THAT(value0.getValue(), Catch::Matchers::WithinRel(expected.getValue(), 1e-2));
        for (int k = 0; k < g0.size(); ++k) {
            REQUIRE_THAT(g0[k], Catch::Matchers::WithinRel(g1[k], 1e-1));
        }
        for (int k = 0; k < h0.size(); ++k) {
            REQUIRE_THAT(
                h0.data()[k],
                Catch::Matchers::WithinRel(h1.data()[k], 1e-1) ||
                    Catch::Matchers::WithinAbs(0, 1e-3));
        }
    }
}

// TODO: Try out sin(x) with periodic boundary cond + autodiff + gradient

TEST_CASE("remeshing using image data")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    int w, h;
    w = 512;
    h = 512;
    Image image(h, w);
    auto displacement_double = [](const double& u, const double& v) -> float {
        return static_cast<float>(10 * sin(M_PI * u) * cos(M_PI * v));
        // return 10 * u;
    };
    auto displacement_dscalar = [](const DScalar& u, const DScalar& v) -> DScalar {
        return 10 * sin(M_PI * u) * cos(M_PI * v);
        // return 10 * u;
    };
    image.set(displacement_double);
    image.save("tryout.exr");
    Image image2(512, 512);
    image2.load("tryout.exr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    auto displacement = [&image2](const DScalar& u, const DScalar& v) -> DScalar {
        return image2.get(u, v);
    };
    auto displacement_image_double = [&image2](const double& u, const double& v) -> double {
        return image2.get(u / 10., v / 10.);
    };
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    TriWild m;
    m.create_mesh(V, F);
    m.set_parameters(0.01, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    m.mesh_improvement(3);

    m.write_displaced_obj("remesh_from_image_linear.obj", displacement_image_double);
}

TEST_CASE("fixed corner")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    TriWild m;
    m.create_mesh(V, F);
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].fixed);
    }
}

TEST_CASE("stripe")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Image image(1024, 1024);
    image.load(
        "/home/yunfan/wildmeshing-toolkit/build_debug/drlin.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    auto displacement_image_double = [&image](const double& u, const double& v) -> double {
        return (image.get(u, v));
    };

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
    REQUIRE(m.invariants(m.get_faces()));

    wmtk::logger().info("#v {}, #f {} ", m.vert_capacity(), m.tri_capacity());
    m.set_image_function(image, WrappingMode::MIRROR_REPEAT);
    m.set_parameters(1, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    m.mesh_improvement(3);
    m.write_displaced_obj("stripe_final.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("implicit points")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/moonface_output.obj", V, F);
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1.); };
    m.set_parameters(0.05, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    int edge_cnt = 0;
    for (auto e : m.get_edges()) {
        auto length2d = m.get_length2d(e.vid(m), e.switch_vertex(m).vid(m));
        auto length3d = m.get_length3d(e.vid(m), e.switch_vertex(m).vid(m));
        if (pow((length2d - length3d), 2) > 1e-6)
            wmtk::logger().info(
                "{} {}",
                m.vertex_attrs[e.vid(m)].pos,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].pos);
        REQUIRE(pow((length2d - length3d), 2) < 1e-6);
        edge_cnt++;
    }
    wmtk::logger().info(edge_cnt);
}

TEST_CASE("quadrature")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    DiffScalarBase::setVariableCount(2);
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/Users/yunfanzhou/Downloads/tmp/input.obj", V, F);
    m.create_mesh(V, F);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1.); };
    m.set_parameters(0.05, displacement, ENERGY_TYPE::EDGE_LENGTH, true);
    int edge_cnt = 0;
    for (auto e : m.get_edges()) {
        auto length2d = m.get_length2d(e.vid(m), e.switch_vertex(m).vid(m));
        auto length3d = m.get_length3d(e.vid(m), e.switch_vertex(m).vid(m));
        auto v1 = m.vertex_attrs[e.vid(m)].pos;
        auto v2 = m.vertex_attrs[e.switch_vertex(m).vid(m)].pos;
        auto length_q = adaptive_gauss_quadrature(m.mesh_parameters.m_project_to_3d, v1, v2, 0.5);
        wmtk::logger().info("edge 2d {} 3d {} guass {}", length2d, length3d, length_q);
    }
}

TEST_CASE("exact length")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    Image image(512, 512);
    image.load(
        "/Users/yunfanzhou/Downloads/tmp/plastic_stripes_Height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    auto displacement_image_double = [&image](const double& u, const double& v) -> double {
        return (image.get(u, v));
    };

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/Users/yunfanzhou/Downloads/tmp/input.obj", V, F);
    m.create_mesh(V, F);
    m.set_image_function(image, WrappingMode::MIRROR_REPEAT);

    m.set_parameters(0.05, displacement, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto e : m.get_edges()) {
        auto length = m.get_length_1ptperpixel(e.vid(m), e.switch_vertex(m).vid(m));
        auto length3d = m.get_length3d(e.vid(m), e.switch_vertex(m).vid(m));
        wmtk::logger().info(
            "length_exact {} length3d {} between {}, {}",
            length,
            length3d,
            e.vid(m),
            e.switch_vertex(m).vid(m));
    }
}

TEST_CASE("mipmap")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    auto displacement_double = [&](const double& u, const double& v) -> float { return 10 * u; };
    Image image(1024, 1024);
    image.set(displacement_double);
    image.save("drlin.exr");

    MipMap mipmap(image);
    REQUIRE(mipmap.level() == 11);
    for (int i = 0; i < mipmap.level(); i++) {
        auto tmp_image = mipmap.get_image(i);
        tmp_image.save(fmt::format("drlin_{:04d}.exr", i));
    }

    // now test the length of the edges using a dummy example
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
    REQUIRE(m.invariants(m.get_faces()));
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    m.set_parameters(
        0.1,
        image,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::MIPMAP,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    m.mesh_improvement(1);
    m.write_displaced_obj("mipmap_out.obj", m.mesh_parameters.m_project_to_3d);
}
