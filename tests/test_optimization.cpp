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

namespace {

/// Central-difference check that `gradient` really is the derivative of `value`, and that
/// `hessian` really is the derivative of `gradient`.
///
/// Every energy here is consumed by a Newton solver: `value` drives the line search while
/// `gradient` and `hessian` pick the step. If the three disagree the solver still "works" --
/// it just takes wrong-sized steps, or rejects good ones, and nothing reports an error. That
/// is how an isotropic hessian and a missing factor of two lived in EnvelopeEnergy, and how
/// DirichletEnergy2D shipped a gradient that is only correct for exactly two cells.
void check_derivatives(
    polysolve::nonlinear::Problem& energy,
    const VectorXd& p,
    const double h = 1e-6,
    const double tol = 1e-5,
    const bool check_hessian = true)
{
    const int n = p.size();

    VectorXd g;
    energy.gradient(p, g);
    REQUIRE(g.size() == n);
    for (int i = 0; i < n; ++i) {
        VectorXd a = p, b = p;
        a[i] -= h;
        b[i] += h;
        const double fd = (energy.value(b) - energy.value(a)) / (2 * h);
        CHECK(std::abs(g[i] - fd) <= tol * std::max(1.0, std::abs(fd)));
    }

    if (!check_hessian) {
        return;
    }
    MatrixXd H;
    energy.hessian(p, H);
    REQUIRE(H.rows() == n);
    REQUIRE(H.cols() == n);
    for (int i = 0; i < n; ++i) {
        VectorXd a = p, b = p;
        a[i] -= h;
        b[i] += h;
        VectorXd ga, gb;
        energy.gradient(a, ga);
        energy.gradient(b, gb);
        const VectorXd fd = (gb - ga) / (2 * h);
        for (int j = 0; j < n; ++j) {
            CHECK(std::abs(H(j, i) - fd[j]) <= 1e-4 * std::max(1.0, std::abs(fd[j])));
        }
    }
}

} // namespace

TEST_CASE("amips_energy_derivatives", "[energies]")
{
    SECTION("2d")
    {
        std::vector<std::array<double, 6>> cells = {
            {{0.4, 0.3, 1, 0, 0.5, 0.9}},
            {{0.4, 0.3, 0.5, 0.9, -0.4, 0.2}},
        };
        optimization::AMIPSEnergy2D energy(cells, 0.75);
        check_derivatives(energy, Vector2d(0.4, 0.3));
    }
    SECTION("3d")
    {
        std::vector<std::array<double, 12>> cells = {
            {{0.3, 0.4, 0.35, 1, 0, 0, 0, 1, 0, 0, 0, 1}},
            {{0.3, 0.4, 0.35, 1, 0, 0, 0, 1, 0, 0.4, 0.4, -0.9}},
        };
        optimization::AMIPSEnergy3D energy(cells, 0.75);
        check_derivatives(energy, Vector3d(0.3, 0.4, 0.35));
    }
}

TEST_CASE("dirichlet_energy_2d_derivatives", "[energies]")
{
    // The pre-existing dirichlet_energy_2d case uses exactly two cells, which is the one
    // count at which a gradient of 2x - sum(y) coincides with the true sum(x - y). Vary the
    // count so the check actually bites.
    const Vector2d p(0.5, 1.0);
    const std::vector<std::array<double, 4>> all = {
        {{p[0], p[1], 0, 0}},
        {{p[0], p[1], 1, 0}},
        {{p[0], p[1], 0.3, -0.7}},
        {{p[0], p[1], -0.6, 0.2}},
    };

    for (size_t n : {size_t(1), size_t(2), size_t(3), size_t(4)}) {
        DYNAMIC_SECTION("cells = " << n)
        {
            std::vector<std::array<double, 4>> cells(all.begin(), all.begin() + n);
            optimization::DirichletEnergy2D energy(cells);
            check_derivatives(energy, p);
        }
    }
}

TEST_CASE("biharmonic_energy_derivatives", "[energies]")
{
    SECTION("2d")
    {
        const Vector2d p0(0.2, 0.9), p1(1, 0), p2(0, 1);
        double M;
        Vector3d L_w;
        optimization::BiharmonicEnergy2D::local_mass_and_stiffness({{p0, p1, p2}}, M, L_w);
        optimization::BiharmonicEnergy2D energy({{p0, p1, p2}}, M, L_w, 0.6);
        check_derivatives(energy, p0);
    }
    SECTION("3d")
    {
        MatrixXd pts(3, 3);
        pts << 0.2, 0.9, 0.1, 1, 0, 0, 0, 1, 0;
        double M;
        VectorXd L_w;
        optimization::BiharmonicEnergy3D::uniform_mass_and_stiffness(pts, M, L_w);
        optimization::BiharmonicEnergy3D energy(pts, M, L_w, 0.6);
        check_derivatives(energy, Vector3d(pts.row(0)));
    }
}

TEST_CASE("energy_sum_derivatives", "[energies]")
{
    // The combination the smoothing actually solves: quality plus stay-on-the-input.
    std::vector<std::array<double, 6>> cells = {
        {{0.4, 0.3, 1, 0, 0.5, 0.9}},
        {{0.4, 0.3, 0.5, 0.9, -0.4, 0.2}},
    };
    auto amips = std::make_shared<optimization::AMIPSEnergy2D>(cells, 1e-4);

    auto env = std::make_shared<SampleEnvelope>(false);
    const std::vector<Eigen::Vector2d> V = {Eigen::Vector2d(-2, 0), Eigen::Vector2d(4, 0)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1)};
    env->init(V, E, 0.1);
    auto envelope = std::make_shared<optimization::EnvelopeEnergy2D>(env, 1.2);

    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(amips, 3.0);
    sum->add_energy(envelope, 0.5);
    check_derivatives(*sum, Vector2d(0.4, 0.3));
}
