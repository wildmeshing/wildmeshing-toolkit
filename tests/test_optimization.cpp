#include <catch2/catch_test_macros.hpp>

#include <igl/readOBJ.h>
#include <igl/writeOFF.h>
#include <filesystem>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
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

TEST_CASE("biharmonic_energy_uniform_3d", "[energies]")
{
    // one-ring
    MatrixXd pts;
    pts.resize(7, 3);
    pts.row(0) = Vector3d(0.5, 0.5, 1); // this vertex is optimized
    pts.row(1) = Vector3d(1, 0, 0);
    pts.row(2) = Vector3d(0.5, 1, 0);
    pts.row(3) = Vector3d(-0.5, 1, 0);
    pts.row(4) = Vector3d(-1, 0, 0);
    pts.row(5) = Vector3d(-0.5, -1, 0);
    pts.row(6) = Vector3d(0.5, -1, 0);
    const Vector3d p0 = pts.row(0);

    double M;
    VectorXd L_w;
    optimization::BiharmonicEnergy3D::uniform_mass_and_stiffness(pts, M, L_w);
    optimization::BiharmonicEnergy3D energy(pts, M, L_w);

    CHECK(energy.is_step_valid(p0, p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[2] > 0);

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

TEST_CASE("biharmonic_energy_cotan_3d", "[energies]")
{
    // one-ring
    MatrixXd pts;
    pts.resize(7, 3);
    pts.row(0) = Vector3d(0.1, 0.3, 1); // this vertex is optimized
    pts.row(1) = Vector3d(1, 0, 0);
    pts.row(2) = Vector3d(0.5, 1, 0);
    pts.row(3) = Vector3d(-0.5, 1, 0);
    pts.row(4) = Vector3d(-1, 0, 0);
    pts.row(5) = Vector3d(-0.5, -1, 0);
    pts.row(6) = Vector3d(0.5, -1, 0);
    const Vector3d p0 = pts.row(0);

    MatrixXi tris;
    tris.resize(6, 3);
    for (size_t i = 0; i < tris.rows(); ++i) {
        size_t v1 = i + 1;
        size_t v2 = (i + 1) % tris.rows() + 1;
        tris.row(i) = Vector3i(0, v1, v2);
    }

    Eigen::SparseMatrix<double> M_glob;
    Eigen::SparseMatrix<double> L_glob;
    optimization::BiharmonicEnergy3D::global_mass_and_stiffness(pts, tris, M_glob, L_glob);
    double M;
    VectorXd L_w;
    optimization::BiharmonicEnergy3D::extract_local_mass_and_stiffness(0, M_glob, L_glob, M, L_w);

    optimization::BiharmonicEnergy3D energy(pts, M, L_w);

    CHECK(energy.is_step_valid(p0, p0));

    {
        VectorXd g;
        energy.gradient(p0, g);
        CHECK(g[2] > 0);

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

TEST_CASE("biharmonic_energy_bunny_3d", "[energies][.]")
{
    // one-ring
    MatrixXd V;
    MatrixXi F;

    igl::readOBJ((data_dir / "models" / "bunny.obj").string(), V, F);

    Eigen::SparseMatrix<double> M_glob;
    Eigen::SparseMatrix<double> L_glob;
    optimization::BiharmonicEnergy3D::global_mass_and_stiffness(V, F, M_glob, L_glob);

    auto m_solver = optimization::create_basic_solver();
    optimization::deactivate_opt_logger();

    size_t n_iters = 1;

    double M;
    VectorXd L_w;
    std::vector<size_t> adj;
    for (size_t i = 0; i < n_iters; ++i) {
        for (size_t vid = 0; vid < V.rows(); ++vid) {
            // get local points and triangles
            optimization::BiharmonicEnergy3D::adjacency_from_stiffness(vid, L_glob, adj);
            if (adj.empty()) {
                continue;
            }
            MatrixXd pts;
            pts.resize(adj.size() + 1, 3);
            pts.row(0) = V.row(vid);
            for (size_t j = 0; j < adj.size(); ++j) {
                pts.row(j + 1) = V.row(adj[j]);
            }

            optimization::BiharmonicEnergy3D::extract_local_mass_and_stiffness(
                vid,
                M_glob,
                L_glob,
                M,
                L_w);

            REQUIRE(pts.rows() == L_w.size());

            optimization::BiharmonicEnergy3D energy(pts, M, L_w);
            auto x = energy.initial_position();
            const double e_before = energy.value(x);
            CHECK_NOTHROW(m_solver->minimize(energy, x));
            V.row(vid) = x;
        }

        // igl::writeOFF(fmt::format("debug_{}.off", i), V, F);
    }
}