#include <catch2/catch_test_macros.hpp>

#include <igl/readOBJ.h>
#include <igl/writeOFF.h>
#include <filesystem>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;

namespace fs = std::filesystem;

const fs::path data_dir = WMTK_DATA_DIR;

TEST_CASE("amips_energy_2d", "[energies]")
{
    const Vector2d p0(0.5, 1e-6);
    std::vector<std::array<double, 6>> cells;
    cells.push_back({{p0[0], p0[1], 0, 0, 1, 0}});

    optimization::AMIPSEnergy2D energy(cells);
    CHECK(energy.is_step_valid(p0, p0));
    CHECK_FALSE(energy.is_step_valid(p0, -p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[1] < 0);

        MatrixXd h;
        CHECK_NOTHROW(energy.hessian(p0, h));
    }
    auto x = energy.initial_position();
    const double e_before = energy.value(x);
    {
        auto linear_solver_params = optimization::basic_linear_solver_params;
        auto nonlinear_solver_params = optimization::basic_nonlinear_solver_params;
        nonlinear_solver_params["max_iterations"] = 100;

        auto m_solver = polysolve::nonlinear::Solver::create(
            nonlinear_solver_params,
            linear_solver_params,
            1,
            opt_logger());
        optimization::deactivate_opt_logger();

        CHECK_NOTHROW(m_solver->minimize(energy, x));
    }
    const double e_after = energy.value(x);
    CHECK(e_after < e_before);
    CHECK(e_after < 2 + 1e-6); // should have perfect quality, i.e. AMIPS=2
}

TEST_CASE("amips_energy_3d", "[energies]")
{
    const Vector3d p0(0.25, 0.25, 1e-2);
    std::vector<std::array<double, 12>> cells;
    cells.push_back({{p0[0], p0[1], p0[2], 0, 0, 0, 0, 1, 0, 1, 0, 0}});

    optimization::AMIPSEnergy3D energy(cells);
    CHECK(energy.is_step_valid(p0, p0));
    CHECK_FALSE(energy.is_step_valid(p0, -p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[2] < 0);

        MatrixXd h;
        CHECK_NOTHROW(energy.hessian(p0, h));
    }
    auto x = energy.initial_position();
    const double e_before = energy.value(x);
    {
        auto linear_solver_params = optimization::basic_linear_solver_params;
        auto nonlinear_solver_params = optimization::basic_nonlinear_solver_params;
        nonlinear_solver_params["max_iterations"] = 100;

        auto m_solver = polysolve::nonlinear::Solver::create(
            nonlinear_solver_params,
            linear_solver_params,
            1,
            opt_logger());
        optimization::deactivate_opt_logger();

        CHECK_NOTHROW(m_solver->minimize(energy, x));
    }
    const double e_after = energy.value(x);
    CHECK(e_after < e_before);
    CHECK(x[2] > p0[2]);
}


TEST_CASE("amips_plus_dirichlet_energy_2d", "[energies]")
{
    const auto VF = utils::examples::tri::edge_region();
    MatrixXd V = VF.V;
    MatrixXi F = VF.F;

    auto write = [&V, &F]() {
        return; // comment out for debug print
        static int wcount = 0;
        MatrixXd V3;
        V3.resize(V.rows(), 3);
        V3.setZero();
        V3.block(0, 0, V.rows(), V.cols()) = V;
        igl::writeOFF(fmt::format("debug_{}.off", wcount++), V3, F);
    };

    V.row(4) = Vector2d(1.8, 0);

    // make vertex 4 the first one in all triangles
    for (int i = 0; i < F.rows(); ++i) {
        int j = 0;
        for (; j < 3; ++j) {
            if (F(i, j) == 4) {
                break;
            }
        }
        if (j < 3) {
            Vector3i f;
            f[0] = F(i, j);
            f[1] = F(i, (j + 1) % 3);
            f[2] = F(i, (j + 2) % 3);
            F.row(i) = f;
        }
    }
    // logger().info("F:\n{}", F);

    // collect all triangles that contain vertex 4
    std::vector<std::array<double, 6>> amips_cells;
    for (int i = 0; i < F.rows(); ++i) {
        if (F(i, 0) != 4) {
            continue;
        }
        std::array<double, 6> c;
        for (size_t j = 0; j < 3; ++j) {
            c[2 * j + 0] = V(F(i, j), 0);
            c[2 * j + 1] = V(F(i, j), 1);
        }
        amips_cells.push_back(c);
    }

    std::vector<std::array<double, 4>> smoothing_cells;
    smoothing_cells.push_back({{V(4, 0), V(4, 1), V(5, 0), V(5, 1)}});
    smoothing_cells.push_back({{V(4, 0), V(4, 1), V(7, 0), V(7, 1)}});

    write();

    auto amips_energy = std::make_shared<optimization::AMIPSEnergy2D>(amips_cells);

    auto total_energy = std::make_shared<optimization::EnergySum>();
    total_energy->add_energy(amips_energy);

    auto x = amips_energy->initial_position();
    const double e_before = amips_energy->value(x);
    {
        auto m_solver = optimization::create_basic_solver();
        optimization::deactivate_opt_logger();
        // CHECK_NOTHROW(m_solver->minimize(*amips_energy, x));
        // CHECK_NOTHROW(m_solver->minimize(*smooth_energy, x));
        CHECK_NOTHROW(m_solver->minimize(*total_energy, x));
    }
    const double e_after = amips_energy->value(x);
    // logger().info("before: {:.4}, after: {:.4}", e_before, e_after);
    CHECK(e_after < e_before);
    V.row(4) = x;
    write();
}


TEST_CASE("exact_distance_energy_2d", "[energies]")
{
    // An L-shaped polyline: one horizontal and one vertical segment meeting at a convex
    // corner. Within each closest-feature region the distance is exactly quadratic, so the
    // energy must pass full FD checks there, with the rank of the hessian reporting the
    // feature: rank one (segment normal) over an interior, isotropic at the corner.
    auto env = std::make_shared<SampleEnvelope>(false);
    const std::vector<Eigen::Vector2d> V = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(4, 0),
        Eigen::Vector2d(4, 4)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1), Eigen::Vector2i(1, 2)};
    env->init(V, E, 0.1);

    const double w = 3.0;
    optimization::ExactDistanceEnergy2D energy(env, w);

    const double h = 1e-6;
    auto fd_check = [&](const Vector2d& p) {
        VectorXd g;
        energy.gradient(p, g);
        MatrixXd H;
        energy.hessian(p, H);
        for (int i = 0; i < 2; ++i) {
            Vector2d a = p, b = p;
            a[i] -= h;
            b[i] += h;
            const double fd = (energy.value(b) - energy.value(a)) / (2 * h);
            CHECK(std::abs(g[i] - fd) <= 1e-5 * std::max(1.0, std::abs(fd)));
            VectorXd ga, gb;
            energy.gradient(a, ga);
            energy.gradient(b, gb);
            for (int j = 0; j < 2; ++j) {
                const double fdh = (gb[j] - ga[j]) / (2 * h);
                CHECK(std::abs(H(j, i) - fdh) <= 1e-4 * std::max(1.0, std::abs(fdh)));
            }
        }
    };

    SECTION("interior region: full FD consistency and rank-one hessian")
    {
        const Vector2d p(2.0, 0.4); // above the horizontal segment, foot in its interior
        fd_check(p);
        MatrixXd H;
        energy.hessian(p, H);
        CHECK((H * Vector2d(1, 0)).norm() <= 1e-9); // free along the segment
        CHECK(std::abs((Vector2d(0, 1).transpose() * H * Vector2d(0, 1)).value() - 2 * w) <= 1e-9);
    }

    SECTION("corner region: full FD consistency and isotropic hessian")
    {
        const Vector2d p(4.5, -0.5); // beyond the corner at (4,0), outside both segments
        fd_check(p);
        MatrixXd H;
        energy.hessian(p, H);
        CHECK((H - 2 * w * MatrixXd::Identity(2, 2)).norm() <= 1e-9);
    }
}

TEST_CASE("exact_distance_energy_3d", "[energies]")
{
    // A 90-degree roof: two triangles sharing the ridge (0,0,0)-(4,0,0), one lying in the
    // z=0 plane (y<0), one in the y=0 plane (z<0). Probes in the (+y,+z) quadrant see the
    // ridge as a convex crease. The three closest-feature kinds get one probe each; within
    // each region the energy is exactly quadratic, so full FD checks must pass, and the
    // hessian's rank reports the feature.
    auto env = std::make_shared<SampleEnvelope>(false);
    const std::vector<Eigen::Vector3d> V = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(4, 0, 0),
        Eigen::Vector3d(2, -3, 0),
        Eigen::Vector3d(2, 0, -3)};
    const std::vector<Eigen::Vector3i> F = {Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(0, 1, 3)};
    env->init(V, F, 0.1);

    const double w = 3.0;
    optimization::ExactDistanceEnergy3D energy(env, w);

    const double h = 1e-6;
    auto fd_check = [&](const Vector3d& p) {
        VectorXd g;
        energy.gradient(p, g);
        MatrixXd H;
        energy.hessian(p, H);
        for (int i = 0; i < 3; ++i) {
            Vector3d a = p, b = p;
            a[i] -= h;
            b[i] += h;
            const double fd = (energy.value(b) - energy.value(a)) / (2 * h);
            CHECK(std::abs(g[i] - fd) <= 1e-5 * std::max(1.0, std::abs(fd)));
            VectorXd ga, gb;
            energy.gradient(a, ga);
            energy.gradient(b, gb);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gb[j] - ga[j]) / (2 * h);
                CHECK(std::abs(H(j, i) - fdh) <= 1e-4 * std::max(1.0, std::abs(fdh)));
            }
        }
    };

    SECTION("face interior: rank one along the face normal")
    {
        const Vector3d p(2.0, -1.0, 0.5); // above the z=0 triangle
        fd_check(p);
        MatrixXd H;
        energy.hessian(p, H);
        CHECK((H * Vector3d(1, 0, 0)).norm() <= 1e-9);
        CHECK((H * Vector3d(0, 1, 0)).norm() <= 1e-9);
        CHECK(
            std::abs((Vector3d(0, 0, 1).transpose() * H * Vector3d(0, 0, 1)).value() - 2 * w) <=
            1e-9);
    }

    SECTION("edge interior: free along the ridge, stiff across")
    {
        const Vector3d p(2.0, 1.0, 1.0); // over the ridge, outside both faces
        fd_check(p);
        MatrixXd H;
        energy.hessian(p, H);
        CHECK((H * Vector3d(1, 0, 0)).norm() <= 1e-9);
        CHECK(
            std::abs((Vector3d(0, 1, 0).transpose() * H * Vector3d(0, 1, 0)).value() - 2 * w) <=
            1e-9);
        CHECK(
            std::abs((Vector3d(0, 0, 1).transpose() * H * Vector3d(0, 0, 1)).value() - 2 * w) <=
            1e-9);
    }

    SECTION("mesh vertex: isotropic")
    {
        const Vector3d p(5.0, 1.0, 1.0); // beyond the ridge end at (4,0,0)
        fd_check(p);
        MatrixXd H;
        energy.hessian(p, H);
        CHECK((H - 2 * w * MatrixXd::Identity(3, 3)).norm() <= 1e-9);
    }
}
