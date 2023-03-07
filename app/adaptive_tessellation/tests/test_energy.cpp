#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/TriQualityUtils.hpp>

#include <wmtk/utils/autodiff.h>
#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <functional>

using namespace wmtk;

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
    Eigen::Vector3d a, b, c;
    a << tri[2] - tri[0], tri[3] - tri[1], 0.;
    b << tri[4] - tri[0], tri[5] - tri[1], 0.;
    c << tri[2] - tri[4], tri[3] - tri[5], 0;
    auto area = (a.cross(b)).norm();
    auto e = a.norm() + b.norm();
    e += c.norm();
    return (std::pow(area, 2) < 1e-5 || std::pow((area / e), 2) < 1e-2);
};
auto array_norm = [](const std::array<double, 6>& a, const std::array<double, 6>& b) {
    double ret;
    for (int i = 0; i < 6; i++) {
        ret += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(ret);
};

TEST_CASE("amips energy")
{
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boudaries(V, F);

    AMIPS amips;
    DofsToPositions dof_to_pos(b, 0);
    DofVector dofx;
    dofx = Eigen::Vector2d(0, 0);
    SECTION("unit equlateral triangle")
    // using the unit equilateral traingle,
    // should have minimum energy 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(1, 0, 1. / 2., sqrt(3) / 2.);

        amips.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 2.0) < 1e-10);
    }
    SECTION("random equlateral triangle")
    // using a random equilateral traingle that doesn't have unit length
    // amips is invariant to triangle edge length, should return minimum energy 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 0, 5. / 2., sqrt(3) * 5 / 2.);

        amips.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 2.0) < 1e-10);
    }
    SECTION("random triangle")
    // any random triangle should have amips energy >= 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 2, 5. / 2., sqrt(3) * 5);

        amips.eval(state, dof_to_pos);
        REQUIRE(state.value > 2);
    }
    SECTION("rototranslation energy")
    {
        // given 2 triangles only differ by rotation,
        // AMIPS energy agianst each other shuld be minimum
        // std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
        // std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
        DofVector dofx = Eigen::Vector2d(-1, 0);
        wmtk::State state1 = {};
        state1.scaling = 1.;
        state1.idx = 0;
        state1.dofx = dofx;
        state1.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(2, 0.5, 0, 8);
        state1.target_triangle = {4., 4., 3.5, 7., -4., 5.};
        amips.eval(state1, dof_to_pos);

        REQUIRE(abs(state1.value - 2.0) < 1e-10);
    }
}

TEST_CASE("symdi energy")
{
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boudaries(V, F);

    SymDi symdi;
    DofsToPositions dof_to_pos(b, 0);
    DofVector dofx;
    dofx = Eigen::Vector2d(0, 0);
    SECTION("unit equlateral triangle")
    // using the unit equilateral traingle,
    // should have minimum energy 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(1, 0, 1. / 2., sqrt(3) / 2.);

        symdi.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 4.0) < 1e-10);
    }
    SECTION("random equlateral triangle")
    // using a random equilateral traingle that doesn't have unit length
    // symdi is variant to triangle edge length, should return energy greater 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 0, 5. / 2., sqrt(3) * 5 / 2.);

        symdi.eval(state, dof_to_pos);
        REQUIRE(state.value > 4.0);
    }
    SECTION("random triangle")
    // any random triangle should have symdi energy >= 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 2, 5. / 2., sqrt(3) * 5);

        symdi.eval(state, dof_to_pos);
        REQUIRE(state.value > 4);
    }
    SECTION("rototranslation energy")
    {
        // given 2 triangles only differ by rotation,
        // AMIPS energy agianst each other shuld be minimum
        // std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
        // std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
        DofVector dofx = Eigen::Vector2d(-1, 0);
        wmtk::State state1 = {};
        state1.scaling = 1.;
        state1.idx = 0;
        state1.dofx = dofx;
        state1.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(2, 0.5, 0, 8);
        state1.target_triangle = {4., 4., 3.5, 7., -4., 5.};
        symdi.eval(state1, dof_to_pos);

        REQUIRE(abs(state1.value - 4.0) < 1e-10);
    }
}

TEST_CASE("test_degenerate")
{
    std::array<double, 6> rand_tri = {27, 35, -14, -46, 26, 33};
    REQUIRE(is_degenerate(rand_tri));
}

TEST_CASE("2 rand tris")
{
    int pass = 0;
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boudaries(V, F);
    DofsToPositions dof_to_pos(b, 0);

    std::array<std::shared_ptr<Energy>, 2> symdi_amips;
    std::shared_ptr<SymDi> symdi = std::make_shared<SymDi>();
    std::shared_ptr<AMIPS> amips = std::make_shared<AMIPS>();
    symdi_amips = {symdi, amips};
    std::array<double, 2> mini_energy = {4., 2.};

    for (int e = 0; e < 2; e++) {
        auto E = symdi_amips[e];
        for (int i = 0; i < 20; i++) {
            std::array<double, 6> target_tri = {0, 0, 0, 0, 0, 0};
            std::array<double, 6> input_tri = {0, 0, 0, 0, 0, 0};
            std::array<std::array<double, 6>*, 2> tris = {&target_tri, &input_tri};
            for (int i = 0; i < 2; i++) {
                while (is_degenerate(*(tris[i])) || is_inverted((*(tris[i])))) {
                    for (int j = 0; j < 6; j++) {
                        (*(tris[i]))[j] = rand() % 100 - 50;
                    }
                }
            }
            // instantiate for newton's method
            wmtk::NewtonMethodInfo nminfo;
            nminfo.curve_id = 0;
            nminfo.target_length = 1;
            nminfo.neighbors.resize(1, 4);
            wmtk::DofVector dofx;
            wmtk::State state = {};
            auto old_tri = input_tri;
            int itr = 0;
            do {
                old_tri = input_tri;
                for (int i = 0; i < 3; i++) {
                    dofx = Eigen::Vector2d(input_tri[i * 2], input_tri[i * 2 + 1]);
                    nminfo.neighbors.row(0) = Eigen::Matrix<double, 1, 4>(
                        input_tri[((i + 1) * 2) % 6],
                        input_tri[((i + 1) * 2 + 1) % 6],
                        input_tri[((i + 2) * 2) % 6],
                        input_tri[((i + 2) * 2 + 1) % 6]);

                    state.target_triangle = {target_tri[i * 2],
                                             target_tri[i * 2 + 1],
                                             target_tri[((i + 1) % 3) * 2],
                                             target_tri[((i + 1) % 3) * 2 + 1],
                                             target_tri[((i + 2) % 3) * 2],
                                             target_tri[((i + 2) % 3) * 2 + 1]};

                    wmtk::newton_method_with_fallback(*E, b, nminfo, dofx, state);
                    E->eval(state, dof_to_pos);

                    input_tri[i * 2] = dofx(0);
                    input_tri[i * 2 + 1] = dofx(1);
                }
                E->eval(state, dof_to_pos);

                std::cout << itr++ << state.gradient.stableNorm() << std::endl;
            } while (state.gradient.stableNorm() > 1e-3);
            E->eval(state, dof_to_pos);
            REQUIRE(abs(state.value - mini_energy[e]) < 1e-2);
        }
    }
}
// TODO: test for edge length error
// TODO: test for accuracy error