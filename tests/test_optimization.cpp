#include <catch2/catch_test_macros.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/ipc.hpp>
#include <ipc/potentials/barrier_potential.hpp>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/BarrierEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

#include <igl/writeOFF.h>

using namespace wmtk;

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

TEST_CASE("dirichlet_energy_2d", "[energies]")
{
    const Vector2d p0(0.5, 1);
    std::vector<std::array<double, 4>> cells;
    cells.push_back({{p0[0], p0[1], 0, 0}});
    cells.push_back({{p0[0], p0[1], 1, 0}});

    optimization::DirichletEnergy2D energy(cells);
    CHECK(energy.is_step_valid(p0, p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[1] > 0);

        MatrixXd h;
        CHECK_NOTHROW(energy.hessian(p0, h));
    }
    auto x = energy.initial_position();
    const double e_before = energy.value(x);
    {
        auto m_solver = optimization::create_basic_solver();
        optimization::deactivate_opt_logger();

        CHECK_NOTHROW(m_solver->minimize(energy, x));
    }
    const double e_after = energy.value(x);
    CHECK(e_after < e_before);
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
    auto smooth_energy = std::make_shared<optimization::DirichletEnergy2D>(smoothing_cells);

    auto total_energy = std::make_shared<optimization::EnergySum>();
    total_energy->add_energy(amips_energy);
    total_energy->add_energy(smooth_energy, 10);

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

TEST_CASE("biharmonic_energy_2d", "[energies]")
{
    const Vector2d p0(0.8, 1); // this vertex is optimized
    const Vector2d p1(0, 0);
    const Vector2d p2(1, 0);

    double M;
    Vector3d L_w;
    optimization::BiharmonicEnergy2D::local_mass_and_stiffness({{p0, p1, p2}}, M, L_w);
    optimization::BiharmonicEnergy2D energy({{p0, p1, p2}}, M, L_w);

    CHECK(energy.is_step_valid(p0, p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[1] > 0);

        MatrixXd h;
        CHECK_NOTHROW(energy.hessian(p0, h));
    }
    auto x = energy.initial_position();
    const double e_before = energy.value(x);
    {
        auto m_solver = optimization::create_basic_solver();
        optimization::deactivate_opt_logger();

        CHECK_NOTHROW(m_solver->minimize(energy, x));
    }
    const double e_after = energy.value(x);
    CHECK(e_after < e_before);
    CHECK(e_after < 1e-5);
}

TEST_CASE("ipc", "[energies][ipc]")
{
    // two edges
    MatrixXd V;
    V.resize(4, 2);
    // e0
    V.row(0) = Vector2d(0, 0);
    V.row(1) = Vector2d(10, 0);
    // e1
    V.row(2) = Vector2d(0, 1e-3);
    V.row(3) = Vector2d(10, 1e-3);

    MatrixXi E;
    E.resize(1, 2);
    E.row(0) = Vector2i(0, 1);
    // E.row(1) = Vector2i(2, 3);

    ipc::CollisionMesh collision_mesh(V, E);

    const double dhat = 1;

    ipc::Candidates candidates; // gather all interesting pairs in here
    candidates.ev_candidates.emplace_back(0, 2);
    candidates.ev_candidates.emplace_back(0, 3);

    ipc::NormalCollisions collisions;
    // collisions.build(collision_mesh, V, dhat);
    collisions.build(candidates, collision_mesh, V, dhat);

    const ipc::BarrierPotential B(dhat);
    double barrier_potential = B(collisions, collision_mesh, V);
    // logger().info("Potential = {}", barrier_potential);

    VectorXd barrier_potential_grad = B.gradient(collisions, collision_mesh, V);
    // logger().info("Grad Potential = \n{}", barrier_potential_grad);

    size_t i = 2;
    Vector2d v2_grad(barrier_potential_grad[2 * i + 0], barrier_potential_grad[2 * i + 1]);
    // logger().info("Grad Potential v2 = {}", v2_grad.transpose());

    CHECK(v2_grad[1] < 0);
    CHECK(v2_grad[0] == 0);

    Eigen::SparseMatrix<double> barrier_potential_hess = B.hessian(collisions, collision_mesh, V);
    Matrix2d v2_hess;
    v2_hess.setZero();
    v2_hess(0, 0) = barrier_potential_hess.coeff(2 * i, 2 * i);
    v2_hess(1, 1) = barrier_potential_hess.coeff(2 * i + 1, 2 * i + 1);
    // logger().info("Hess v2 = \n{}", v2_hess);

    CHECK(v2_hess(0, 0) == 0);
    CHECK(v2_hess(0, 1) == 0);
    CHECK(v2_hess(1, 0) == 0);
    CHECK(v2_hess(1, 1) > 0);
    // logger().info("Hess full = \n{}", MatrixXd(barrier_potential_hess));
}

TEST_CASE("barrier_energy_2d", "[energies][ipc]")
{
    // 4 edges, two top two bottom
    MatrixXd V;
    V.resize(6, 2);
    // bottom two edges
    V.row(0) = Vector2d(0, 0);
    V.row(1) = Vector2d(10, 0);
    V.row(2) = Vector2d(20, 0);
    // top two edges
    V.row(3) = Vector2d(0, 1e-3);
    V.row(4) = Vector2d(10, 1e-3);
    V.row(5) = Vector2d(20, 1e-3);

    MatrixXi E;
    E.resize(4, 2);
    E.row(0) = Vector2i(0, 1);
    E.row(1) = Vector2i(1, 2);
    E.row(2) = Vector2i(3, 4);
    E.row(3) = Vector2i(4, 5);


    optimization::BarrierEnergy2D energy(V, E, 4, 1);
    const Vector2d p0 = energy.initial_position();
    CHECK(energy.is_step_valid(p0, p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        REQUIRE(g.size() == 2);
        CHECK(g[1] < 0);

        MatrixXd h;
        CHECK_NOTHROW(energy.hessian(p0, h));
    }
    auto x = energy.initial_position();
    const double e_before = energy.value(x);
    CHECK(e_before > 0);
    {
        auto linear_solver_params = optimization::basic_linear_solver_params;
        auto nonlinear_solver_params = optimization::basic_nonlinear_solver_params;
        nonlinear_solver_params["max_iterations"] = 100;

        auto solver = polysolve::nonlinear::Solver::create(
            nonlinear_solver_params,
            linear_solver_params,
            1,
            opt_logger());
        optimization::deactivate_opt_logger();

        CHECK_NOTHROW(solver->minimize(energy, x));
    }
    const double e_after = energy.value(x);
    CHECK(e_after < e_before);
    {
        VectorXd g;
        energy.gradient(x, g);
        CHECK(g[1] < 1e-6);
    }
}