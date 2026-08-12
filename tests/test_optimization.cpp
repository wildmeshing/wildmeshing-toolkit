#include <catch2/catch_test_macros.hpp>

#include <igl/readOBJ.h>
#include <igl/writeOFF.h>
#include <filesystem>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
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

    SECTION("max_step_size clamps at the feature boundary")
    {
        // A step staying over one segment interior is not clamped.
        CHECK(energy.max_step_size(Vector2d(1.0, 0.4), Vector2d(2.0, 0.4)) == 1.0);

        // A step from over the horizontal segment toward the vertical one crosses the
        // inner bisector at x = 3.6, where both segments are 0.4 away and the closest
        // feature switches (the C0 kink of the distance on the concave side). The clamp
        // must land just past that boundary, not sail to the far end.
        const Vector2d a(3.0, 0.4);
        const Vector2d b(6.0, 0.4);
        const double alpha = energy.max_step_size(a, b);
        CHECK(alpha < 1.0);
        const double x_land = a[0] + alpha * (b[0] - a[0]);
        CHECK(x_land >= 3.6);
        CHECK(x_land <= 3.6 + 1e-4 * (b[0] - a[0]));
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

TEST_CASE("spring_energy_2d_fd_consistency", "[energies]")
{
    // The spring is an exact quadratic to a fixed target, so unlike the envelope energy --
    // whose hessian is deliberately the Gauss-Newton model, not the derivative of the
    // gradient -- ALL THREE callbacks must pass a finite-difference check against each
    // other, to rounding.
    const Vector2d target(1.5, -0.5);
    const double w = 3.0;
    optimization::SpringEnergy2D energy(target, w);

    const std::vector<Vector2d> probes = {
        Vector2d(1.0, 0.30),
        Vector2d(2.0, -0.45),
        Vector2d(-0.7, 1.10),
    };
    const double h = 1e-6;

    SECTION("gradient is the derivative of the value")
    {
        for (const Vector2d& p : probes) {
            VectorXd g;
            energy.gradient(p, g);
            for (int i = 0; i < 2; ++i) {
                Vector2d a = p, b = p;
                a[i] -= h;
                b[i] += h;
                const double fd = (energy.value(b) - energy.value(a)) / (2 * h);
                CHECK(std::abs(g[i] - fd) <= 1e-5 * std::max(1.0, std::abs(fd)));
            }
        }
    }

    SECTION("hessian is the derivative of the gradient")
    {
        for (const Vector2d& p : probes) {
            MatrixXd H;
            energy.hessian(p, H);
            for (int i = 0; i < 2; ++i) {
                Vector2d a = p, b = p;
                a[i] -= h;
                b[i] += h;
                VectorXd ga, gb;
                energy.gradient(a, ga);
                energy.gradient(b, gb);
                for (int j = 0; j < 2; ++j) {
                    const double fd = (gb[j] - ga[j]) / (2 * h);
                    CHECK(std::abs(H(j, i) - fd) <= 1e-5 * std::max(1.0, std::abs(fd)));
                }
            }
        }
    }
}

TEST_CASE("envelope_energy_2d_derivatives", "[energies]")
{
    // value(), gradient() and hessian() have to describe the SAME function. Nothing checked
    // that, and they had drifted apart: the gradient was half the derivative of the value,
    // and the hessian was isotropic.
    //
    // The isotropy is the one with teeth. The energy is w * (distance to the input)^2, so
    // moving ALONG the input costs nothing and the true curvature in that direction is zero;
    // the Gauss-Newton hessian 2w * n n^T is rank one. An isotropic 2w * I instead applies
    // the full normal stiffness tangentially, and since a surface vertex's quality term is
    // weighted 1e-4 against it, the tangential Newton step is divided by a number that does
    // not belong there and the vertex cannot slide along the curve it sits on.
    auto env = std::make_shared<SampleEnvelope>(false);
    const std::vector<Eigen::Vector2d> V = {Eigen::Vector2d(0, 0), Eigen::Vector2d(4, 0)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1)};
    env->init(V, E, 0.1);

    const double w = 3.0;
    optimization::EnvelopeEnergy2D energy(env, w);

    // Probes off the segment but away from its endpoints, where the distance field is smooth.
    const std::vector<Vector2d> probes = {
        Vector2d(1.0, 0.30),
        Vector2d(2.0, -0.45),
        Vector2d(3.0, 0.20),
    };

    SECTION("the gradient is the derivative of the value")
    {
        const double h = 1e-6;
        for (const Vector2d& p : probes) {
            VectorXd g;
            energy.gradient(p, g);
            for (int i = 0; i < 2; ++i) {
                Vector2d a = p, b = p;
                a[i] -= h;
                b[i] += h;
                const double fd = (energy.value(b) - energy.value(a)) / (2 * h);
                CHECK(std::abs(g[i] - fd) <= 1e-5 * std::max(1.0, std::abs(fd)));
            }
        }
    }

    SECTION("the hessian does not resist motion along the input")
    {
        // The segment runs along x, so sliding in x leaves the distance unchanged and the
        // hessian must have no component there. An isotropic hessian fails this with the
        // full 2w.
        const Vector2d tangent(1, 0);
        for (const Vector2d& p : probes) {
            MatrixXd h;
            energy.hessian(p, h);
            CHECK((h * tangent).norm() <= 1e-9);

            // ... while the normal direction carries the full 2w curvature.
            const Vector2d normal(0, 1);
            CHECK(std::abs((normal.transpose() * h * normal).value() - 2 * w) <= 1e-9);
        }
    }

    SECTION("the hessian is positive semi-definite")
    {
        for (const Vector2d& p : probes) {
            MatrixXd h;
            energy.hessian(p, h);
            Eigen::SelfAdjointEigenSolver<MatrixXd> es(h);
            CHECK(es.eigenvalues().minCoeff() >= -1e-12);
        }
    }
}
