#include <wmtk/components/topological_offset/ExactBandIntegral.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <catch2/catch_test_macros.hpp>

#include "exact_band_integral_data.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace wmtk::components::topological_offset;

/**
 * ExactBandIntegral against its Python reference (exact_integral/exact_tri.py, exact_mesh.py).
 * The data in exact_band_integral_data.hpp is the Python's output, bit exact (hex floats):
 *  1. one triangle, each region configuration of exact_tri's tests: the port against
 *     exact_mesh.integrate_mesh (its spec) and exact_tri.integrate (independent: no mesh
 *     regions), and the same branch choices (near-parallel series edge counts);
 *  2. additivity over the 8 red children (exact cells only: a rule-(a) piece is a bound);
 *  3. stored band cells of five offset runs against integrate_mesh on the full input;
 *  4. flat cells are 0.
 * Units: I = int_t d dV = value(t, 0); relative errors |I - I_ref| / I_ref.
 */

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;

Tet tet_of(const double* T)
{
    Tet t;
    for (int i = 0; i < 4; ++i) t[size_t(i)] = Vec3(T[3 * i], T[3 * i + 1], T[3 * i + 2]);
    return t;
}

ExactBandIntegral one_triangle(const double* P)
{
    Eigen::MatrixXd V(3, 3);
    for (int i = 0; i < 3; ++i) V.row(i) << P[3 * i], P[3 * i + 1], P[3 * i + 2];
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    return ExactBandIntegral(V, F);
}

ExactBandIntegral of_case(const exact_band_data::RealCase& c)
{
    Eigen::MatrixXd V(c.nV, 3);
    for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
    Eigen::MatrixXi F(c.nF, 3);
    for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
    return ExactBandIntegral(V, F);
}

double rel(const double a, const double b)
{
    return std::abs(a - b) / std::max(std::abs(b), 1e-300);
}

/// The 8 children of the edge-midpoint (red) subdivision (test_exact_tri.red_children).
std::array<Tet, 8> red_children(const Tet& T)
{
    const auto M = [&](int i, int j) { return Vec3(0.5 * (T[size_t(i)] + T[size_t(j)])); };
    std::array<Tet, 8> k;
    k[0] = {T[0], M(0, 1), M(0, 2), M(0, 3)};
    k[1] = {T[1], M(0, 1), M(1, 2), M(1, 3)};
    k[2] = {T[2], M(0, 2), M(1, 2), M(2, 3)};
    k[3] = {T[3], M(0, 3), M(1, 3), M(2, 3)};
    const Vec3 a = M(0, 2), b = M(1, 3);
    const std::array<Vec3, 4> ring = {M(0, 1), M(1, 2), M(2, 3), M(0, 3)};
    for (int i = 0; i < 4; ++i) k[size_t(4 + i)] = {a, b, ring[size_t(i)], ring[size_t((i + 1) % 4)]};
    return k;
}

int64_t series(const ExactBandIntegral::Stats& s)
{
    return s.series_short + s.series_long;
}

} // namespace

TEST_CASE("exact-band-integral-one-triangle", "[offset][exact_band]")
{
    // Bounds: 1e-12 on the value, 1e-12 on additivity (MEASURED on the mac-mini: <= 2.7e-14 and
    // <= 4.5e-14), except two cells whose conditioning, not the port, sets the error:
    //  - "far away (D/L ~ 100)": the Euler sums lose about eps (D/L)^2 = 2e-12 (test_exact_tri.py
    //    "far"); measured 9.1e-12 against both Python values, additivity 2.1e-11;
    //  - "tiny t (1e-6) on P's edge": the children's midpoints are rounded at eps |x| ~ 1e-16,
    //    1e-10 of the cell's size; additivity measured 4.7e-11 (the value itself agrees to 0).
    const auto tol = [](const std::string& label, bool additivity) {
        if (label.rfind("far away", 0) == 0) return 1e-10;
        if (additivity && label.rfind("tiny t", 0) == 0) return 1e-9;
        return 1e-12;
    };
    std::map<std::string, std::array<double, 4>> worst; // port, vs exact_tri, additivity, series
    for (const auto& c : exact_band_data::tri_cells) {
        const ExactBandIntegral eb = one_triangle(c.P);
        const Tet T = tet_of(c.T);
        ExactBandIntegral::Stats st;
        const double I = eb.value(T, 0., &st);
        INFO(c.label);
        CHECK(rel(I, c.Im) <= tol(c.label, false));
        CHECK(rel(I, c.Ix) <= tol(c.label, false));
        // the near-parallel series is exercised where the Python exercises it (the exact counts
        // can differ by faces whose Euler weight h is rounding noise, see the real-cells case)
        CHECK((series(st) > 0) == (c.nseries > 0));
        CHECK(st.rule_a_vol == 0.); // one triangle: the regions tile space
        double sum = 0.;
        for (const Tet& k : red_children(T)) sum += eb.value(k, 0.);
        CHECK(rel(sum, I) <= tol(c.label, true));
        auto& w = worst[c.label];
        w[0] = std::max(w[0], rel(I, c.Im));
        w[1] = std::max(w[1], rel(I, c.Ix));
        w[2] = std::max(w[2], rel(sum, I));
        w[3] += double(series(st));
    }
    std::printf("exact-band-integral-one-triangle: max relative error per configuration\n");
    std::printf("  %-30s %9s %9s %9s %7s\n", "", "vs mesh", "vs tri", "additiv.", "series");
    for (const auto& [k, w] : worst) {
        std::printf("  %-30s %9.1e %9.1e %9.1e %7.0f\n", k.c_str(), w[0], w[1], w[2], w[3]);
    }
}

TEST_CASE("exact-band-integral-real-cells", "[offset][exact_band]")
{
    std::printf("exact-band-integral-real-cells: max relative error against the Python\n");
    for (const auto& c : exact_band_data::real_cases) {
        const ExactBandIntegral eb = of_case(c);
        double worst = 0., worst_add = 0.;
        int n_add = 0, n_rule_a = 0;
        std::vector<double> J(size_t(c.ncells));
        for (int i = 0; i < c.ncells; ++i) {
            const auto& cell = c.cells[i];
            const Tet T = tet_of(cell.T);
            ExactBandIntegral::Stats st;
            const double I = eb.value(T, 0., &st);
            J[size_t(i)] = eb.value(T, c.delta);
            INFO(c.name << " cell " << i << " (" << cell.cat << ")");
            CHECK(rel(I, cell.I) <= 1e-12);
            // J = I - delta vol cancels in the band; its error is measured against I
            CHECK(std::abs(J[size_t(i)] - (cell.I - c.delta * cell.vol)) <= 1e-12 * cell.I);
            // Rule-(a) volume up to rounding slivers (test_exact_mesh.py's SLIVER = 1e-12 vol). The
            // branch counts are NOT compared: a piece face parallel to an input edge line up to
            // rounding (e.n ~ 1e-18) has Euler weight h at _through()'s tolerance, and rounding
            // decides whether it is skipped (measured: t100026 cell 2, 4 line faces here, 6 in the
            // Python, the two extra with |e.n| < 2e-18); its term is rounding noise either way.
            CHECK(std::abs(st.rule_a_vol - cell.rule_a_vol) <= 1e-12 * cell.vol);
            worst = std::max(worst, rel(I, cell.I));
            n_rule_a += cell.rule_a_vol > 0.;
            if (st.rule_a_vol > 0.) continue;
            // additivity, exact families only
            double sum = 0.;
            bool exact = true;
            for (const Tet& k : red_children(T)) {
                ExactBandIntegral::Stats sk;
                sum += eb.value(k, 0., &sk);
                exact = exact && sk.rule_a_vol == 0.;
            }
            if (!exact) continue;
            CHECK(rel(sum, I) <= 1e-12);
            worst_add = std::max(worst_add, rel(sum, I));
            ++n_add;
        }
        std::printf(
            "  %-13s %2d cells (%d with rule (a)): max %.1e; additivity on %d exact families: max %.1e\n",
            c.name,
            c.ncells,
            n_rule_a,
            worst,
            n_add,
            worst_add);
        // value() is const and stateless: four threads at once give the same bits
        std::vector<std::vector<double>> Jt(4, std::vector<double>(size_t(c.ncells)));
        std::vector<std::thread> th;
        for (int t = 0; t < 4; ++t) {
            th.emplace_back([&, t] {
                for (int i = 0; i < c.ncells; ++i) {
                    Jt[size_t(t)][size_t(i)] = eb.value(tet_of(c.cells[i].T), c.delta);
                }
            });
        }
        for (auto& x : th) x.join();
        for (int t = 0; t < 4; ++t) CHECK(Jt[size_t(t)] == J);
    }
}

TEST_CASE("exact-band-integral-flat", "[offset][exact_band]")
{
    const double P0[9] = {0., 0., 0., 1., 0., 0., 0.3, 0.8, 0.};
    const ExactBandIntegral eb = one_triangle(P0);
    const Vec3 p(0.3, -0.2, 0.7), q(-0.5, 0.4, 0.1), r(0.9, 0.6, -0.3);
    const std::vector<Tet> flats = {
        {p, q, r, Vec3(0.3 * p + 0.3 * q + 0.4 * r + 0.2 * (q - p))}, // 4 coplanar points
        {p, q, Vec3(0.5 * (p + q)), r}, // 3 collinear + 1
        {p, p, q, r}, // two coincident vertices
        {p, p, p, p}, // all coincident
        {Vec3(0.1, 0.1, 0), Vec3(0.9, 0.05, 0), Vec3(0.3, 0.7, 0), Vec3(0.4, 0.3, 0)}, // on P's plane
    };
    for (const Tet& T : flats) {
        CHECK(eb.value(T, 0.) == 0.);
        CHECK(eb.value(T, 0.5) == 0.);
        Vec3 G;
        Eigen::Matrix3d H;
        CHECK(eb.value_grad_hess(T, 1, 0.5, G, H) == 0.);
        CHECK(G.isZero(0.));
        CHECK(H.isZero(0.));
    }
}

namespace {

/// |a - b| / |b| for the derivative references (Frobenius norm for H).
double rel_vec(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    return (a - b).norm() / std::max(b.norm(), 1e-300);
}

/// Parity of one value_grad_hess() call against a Python reference: (J, G, H) relative errors, J
/// against |J_ref| + delta vol (J = I - delta vol cancels in the band).
std::array<double, 3> deriv_parity(
    const ExactBandIntegral& eb,
    const Tet& T,
    const exact_band_data::DerivRef& r,
    const double delta,
    const double vol)
{
    Vec3 G;
    Eigen::Matrix3d H;
    const double J = eb.value_grad_hess(T, r.kv, delta, G, H);
    const Eigen::Map<const Eigen::Vector3d> Gr(r.G);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Hr(r.H);
    return {
        std::abs(J - r.J) / (std::abs(r.J) + delta * vol),
        rel_vec(G, Gr),
        rel_vec(Eigen::Map<const Eigen::VectorXd>(H.data(), 9),
                Eigen::Map<const Eigen::VectorXd>(Eigen::Matrix3d(Hr).data(), 9))};
}

/// test_exact_mesh_derivs.fd(): relative errors of the gradient and Hessian against central
/// differences (gradient from value(), Hessian from value_grad_hess()) at each step h L,
/// h = 1e-3 .. 1e-7, L the longest edge; and at h = 1e-5 the kink measure |H_fwd - H_bwd| / |H|
/// with the one-sided error of H d along the inward direction d at h = 1e-5 and 1e-6.
struct FD
{
    std::array<std::array<double, 2>, 5> rows; // per h: gradient, Hessian
    double kink = 0., inward5 = 0., inward6 = 0.;
};
FD fd(const ExactBandIntegral& eb, const Tet& T, const int kv, const double delta)
{
    constexpr double HS[5] = {1e-3, 1e-4, 1e-5, 1e-6, 1e-7};
    double L = 0.;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < i; ++j) L = std::max(L, (T[size_t(i)] - T[size_t(j)]).norm());
    }
    Vec3 G;
    Eigen::Matrix3d H;
    eb.value_grad_hess(T, kv, delta, G, H);
    FD out;
    for (int ih = 0; ih < 5; ++ih) {
        const double hh = HS[ih] * L;
        Vec3 Gfd;
        Eigen::Matrix3d Hfd, Hf, Hb;
        for (int i = 0; i < 3; ++i) {
            double jv[2];
            Vec3 gv[2];
            for (int s = 0; s < 2; ++s) {
                Tet T2 = T;
                T2[size_t(kv)][i] += (s == 0 ? 1. : -1.) * hh;
                jv[s] = eb.value(T2, delta);
                Eigen::Matrix3d Hd;
                eb.value_grad_hess(T2, kv, delta, gv[s], Hd);
            }
            Gfd[i] = (jv[0] - jv[1]) / (2 * hh);
            Hfd.col(i) = (gv[0] - gv[1]) / (2 * hh);
            Hf.col(i) = (gv[0] - G) / hh;
            Hb.col(i) = (G - gv[1]) / hh;
        }
        out.rows[size_t(ih)] = {(Gfd - G).norm() / G.norm(), (Hfd - H).norm() / H.norm()};
        if (ih == 2) {
            out.kink = (Hf - Hb).norm() / H.norm();
            Vec3 d = Vec3::Zero();
            for (int i = 0; i < 4; ++i) {
                if (i != kv) d += T[size_t(i)] / 3.;
            }
            d = (d - T[size_t(kv)]).normalized();
            for (const double hd : {1e-5, 1e-6}) {
                Tet T2 = T;
                T2[size_t(kv)] += hd * L * d;
                Vec3 G2;
                Eigen::Matrix3d H2;
                eb.value_grad_hess(T2, kv, delta, G2, H2);
                const double err = ((G2 - G) / (hd * L) - H * d).norm() / H.norm();
                (hd == 1e-5 ? out.inward5 : out.inward6) = err;
            }
        }
    }
    return out;
}

/// test_exact_mesh_derivs.KINK: |H_fwd - H_bwd| / |H| at h = 1e-5 above this: grad J has a kink
/// at x (a kink of d on a face of t through x), and H is the one-sided limit for x moving into t.
constexpr double KINK = 1e-3;

struct FDSummary
{
    int smooth = 0, kinks = 0;
    double grad = 0., hess = 0., inward = 0.; // max over corners: min over h; inward at h = 1e-6
    void add(const FD& f)
    {
        if (f.kink > KINK) {
            ++kinks;
            inward = std::max(inward, f.inward6);
            return;
        }
        ++smooth;
        double g = 1e300, h = 1e300;
        for (const auto& r : f.rows) g = std::min(g, r[0]), h = std::min(h, r[1]);
        grad = std::max(grad, g);
        hess = std::max(hess, h);
    }
};

} // namespace

TEST_CASE("exact-band-integral-derivatives", "[offset][exact_band]")
{
    // Parity with exact_mesh.value_grad_hess (one corner per cell), and finite differences at every
    // corner as test_exact_mesh_derivs.py does. Python's own finite-difference numbers on the
    // stored cases (2026-10-12, smooth corners, min over h, max): gradient 4.8e-8, Hessian 2.1e-6;
    // kink corners, inward one-sided error at h = 1e-6: max 5.3e-3. The bounds below have margin
    // over those and over this port's measured numbers (printed).
    std::printf("exact-band-integral-derivatives\n");
    {
        std::array<double, 3> worst = {0., 0., 0.}, worst_far = {0., 0., 0.};
        FDSummary fs, fs_far;
        for (const auto& r : exact_band_data::tri_derivs) {
            const auto& c = exact_band_data::tri_cells[r.cell];
            const ExactBandIntegral eb = one_triangle(c.P);
            const Tet T = tet_of(c.T);
            const double vol = std::abs((T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]))) / 6.;
            const auto e = deriv_parity(eb, T, r, exact_band_data::tri_delta, vol);
            const bool far = std::string(c.label).rfind("far away", 0) == 0;
            INFO(c.label << " kv " << r.kv);
            // far away: the Euler sums' eps (D/L)^2 conditioning, as for the value
            CHECK(e[0] <= (far ? 1e-10 : 1e-12));
            CHECK(e[1] <= (far ? 1e-9 : 1e-11));
            CHECK(e[2] <= (far ? 1e-7 : 1e-10));
            auto& w = far ? worst_far : worst;
            for (int i = 0; i < 3; ++i) w[size_t(i)] = std::max(w[size_t(i)], e[size_t(i)]);
            for (int kv = 0; kv < 4; ++kv) {
                FD f = fd(eb, T, kv, exact_band_data::tri_delta);
                // far away: G carries the eps (D/L)^2 rounding, which 1/h amplifies (MEASURED:
                // Hessian FD error 5.5e-6 at h = 1e-3 rising to 4e-2 at 1e-7), so the kink measure
                // (2-4e-3 there) is that noise, not a kink: no classification, min over h
                if (far) f.kink = 0.;
                (far ? fs_far : fs).add(f);
            }
        }
        std::printf(
            "  one triangle: parity J %.1e G %.1e H %.1e (far away: %.1e %.1e %.1e)\n",
            worst[0], worst[1], worst[2], worst_far[0], worst_far[1], worst_far[2]);
        std::printf(
            "  one triangle, all corners: %d smooth: FD min over h, max: gradient %.1e Hessian %.1e; "
            "%d kink corners: inward one-sided (h=1e-6) max %.1e | far away, %d corners: %.1e %.1e\n",
            fs.smooth, fs.grad, fs.hess, fs.kinks, fs.inward, fs_far.smooth, fs_far.grad, fs_far.hess);
        CHECK(fs.grad <= 1e-7);
        CHECK(fs.hess <= 1e-4);
        CHECK(fs.inward <= 2e-2);
        CHECK(fs_far.grad <= 1e-6);
        CHECK(fs_far.hess <= 1e-3);
    }
    for (const auto& c : exact_band_data::real_cases) {
        const ExactBandIntegral eb = of_case(c);
        std::array<double, 3> worst = {0., 0., 0.};
        for (int i = 0; i < c.nderivs; ++i) {
            const auto& r = c.derivs[i];
            const auto& cell = c.cells[r.cell];
            const auto e = deriv_parity(eb, tet_of(cell.T), r, c.delta, cell.vol);
            INFO(c.name << " cell " << r.cell << " kv " << r.kv);
            CHECK(e[0] <= 1e-12);
            CHECK(e[1] <= 1e-11);
            CHECK(e[2] <= 1e-10);
            for (int k = 0; k < 3; ++k) worst[size_t(k)] = std::max(worst[size_t(k)], e[size_t(k)]);
        }
        FDSummary fs;
        for (int i = 0; i < c.ncells; ++i) {
            for (int kv = 0; kv < 4; ++kv) fs.add(fd(eb, tet_of(c.cells[i].T), kv, c.delta));
        }
        std::printf(
            "  %-13s parity (%d cells) J %.1e G %.1e H %.1e | FD, %d smooth corners: gradient %.1e "
            "Hessian %.1e; %d kink corners: inward %.1e\n",
            c.name, c.nderivs, worst[0], worst[1], worst[2], fs.smooth, fs.grad, fs.hess, fs.kinks,
            fs.inward);
        CHECK(fs.grad <= 1e-7);
        CHECK(fs.hess <= 1e-4);
        CHECK(fs.inward <= 2e-2);
    }
}

TEST_CASE("exact-band-integral-energy", "[offset][exact_band]")
{
    // EXPERIMENTAL_band_volume_rule "exact" in the smoother: BandVolumeEnergy3D's value is
    // sum_t J_t / delta (ExactBandIntegral::value(), what band_cell_term() reads), its gradient
    // and Hessian sum_t of value_grad_hess() / delta in the moving vertex x; equal bit for bit to
    // those sums, whatever the order of the calls (each keeps the last point's), and consistent
    // with finite differences. A ring of cells around a point beyond an edge of the stored cube.
    const auto& c = exact_band_data::real_cases[0];
    REQUIRE(std::string(c.name) == "cube_out");
    Eigen::MatrixXd V(c.nV, 3);
    for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
    Eigen::MatrixXi F(c.nF, 3);
    for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
    const auto eb = std::make_shared<const ExactBandIntegral>(V, F);
    const double delta = c.delta;
    const auto tris = std::make_shared<const InputTriangles>(V, F);
    const Vec3 ctr(0.52, 0.1, 0.515); // beyond the edge x = z = 0.5
    const double rad = 0.02;
    std::vector<BandVolumeEnergy3D::Cell> cells;
    for (const double sx : {-1., 1.}) {
        for (const double sy : {-1., 1.}) {
            BandVolumeEnergy3D::Cell cl;
            cl.q1 = ctr + sx * rad * Vec3::UnitX();
            cl.q2 = ctr + sy * rad * Vec3::UnitY();
            cl.q3 = ctr - rad * Vec3::UnitZ();
            cells.push_back(cl);
        }
    }
    BandVolumeEnergy3D energy(tris, cells, ctr, delta);
    energy.set_exact_integral(eb, delta);
    const Eigen::VectorXd xv = ctr + Vec3(0.002, 0.001, 0.004);
    const Vec3 x = xv;
    double want = 0.;
    Vec3 gwant = Vec3::Zero();
    Eigen::Matrix3d hwant = Eigen::Matrix3d::Zero();
    for (auto cl : cells) {
        // the constructor's orientation: q2, q3 swapped where the volume at ctr is negative
        if ((cl.q1 - ctr).dot((cl.q2 - cl.q1).cross(cl.q3 - cl.q1)) < 0.) std::swap(cl.q2, cl.q3);
        want += eb->value({{x, cl.q1, cl.q2, cl.q3}}, delta) / delta;
        Vec3 gc;
        Eigen::Matrix3d hc;
        eb->value_grad_hess({{x, cl.q1, cl.q2, cl.q3}}, 0, delta, gc, hc);
        gwant += gc / delta;
        hwant += hc / delta;
    }
    Eigen::VectorXd g;
    Eigen::MatrixXd H;
    energy.gradient(xv, g); // derivatives first, then the value, then again: the caches agree
    CHECK(energy.value(xv) == want);
    energy.hessian(xv, H);
    CHECK(Vec3(g) == gwant);
    CHECK(Eigen::Matrix3d(H) == hwant);
    CHECK(energy.value(xv) == want);
    const double h = 1e-7;
    for (int k = 0; k < 3; ++k) {
        Eigen::VectorXd xp = xv, xm = xv;
        xp[k] += h;
        xm[k] -= h;
        CHECK(std::abs((energy.value(xp) - energy.value(xm)) / (2. * h) - g[k]) <= 1e-6 * g.norm());
        Eigen::VectorXd gp, gm;
        energy.gradient(xp, gp);
        energy.gradient(xm, gm);
        for (int j = 0; j < 3; ++j) {
            CHECK(std::abs((gp[j] - gm[j]) / (2. * h) - H(j, k)) <= 1e-5 * H.norm());
        }
    }
}

/**
 * Hidden benchmark and parity sweep: EXACT_BAND_BENCH_DIR=<dir> ./wmtk_test_topological_offset
 * "[.bench]". Reads <dir>/<case>.bin (gen_data.py: the full input, every stored sample cell and
 * random band cells, each with the Python's integrate_mesh), prints the parity per category,
 * the time per cell, its split, and the corner bound's time on the same cells.
 */
TEST_CASE("exact-band-integral-bench", "[.bench]")
{
    const char* dir = std::getenv("EXACT_BAND_BENCH_DIR");
    if (!dir) {
        WARN("EXACT_BAND_BENCH_DIR not set");
        return;
    }
    const char* cats[] = {"touch", "front", "random", "medial"};
    using clk = std::chrono::steady_clock;
    for (const char* name : {"cube_out", "cylinder_out", "prism_in", "slot", "t100026"}) {
        std::ifstream in(std::string(dir) + "/" + name + ".bin", std::ios::binary);
        REQUIRE(in.good());
        int32_t hdr[4];
        double delta;
        in.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
        in.read(reinterpret_cast<char*>(&delta), sizeof(delta));
        const int nV = hdr[0], nF = hdr[1], nS = hdr[2], nR = hdr[3];
        std::vector<double> Vb(static_cast<size_t>(3 * nV)), Ts(static_cast<size_t>(12 * nS)), Is(static_cast<size_t>(nS)), Tr(static_cast<size_t>(12 * nR)),
            Ir(static_cast<size_t>(nR));
        std::vector<int32_t> Fb(static_cast<size_t>(3 * nF)), cat(static_cast<size_t>(nS));
        const auto rd = [&](auto& v) {
            in.read(reinterpret_cast<char*>(v.data()), std::streamsize(v.size() * sizeof(v[0])));
        };
        rd(Vb);
        rd(Fb);
        rd(Ts);
        rd(Is);
        rd(cat);
        rd(Tr);
        rd(Ir);
        REQUIRE(in.good());
        Eigen::MatrixXd V(nV, 3);
        Eigen::MatrixXi F(nF, 3);
        for (int i = 0; i < nV; ++i) V.row(i) << Vb[size_t(3 * i)], Vb[size_t(3 * i + 1)], Vb[size_t(3 * i + 2)];
        for (int i = 0; i < nF; ++i) F.row(i) << Fb[size_t(3 * i)], Fb[size_t(3 * i + 1)], Fb[size_t(3 * i + 2)];
        auto t0 = clk::now();
        const ExactBandIntegral eb(V, F);
        const double t_build = std::chrono::duration<double>(clk::now() - t0).count();
        const auto tris = std::make_shared<InputTriangles>(V, F);

        std::printf("== %s: %d triangles (build %.2f ms), delta %.6g\n", name, nF, 1e3 * t_build, delta);
        // parity
        double worst_cat[4] = {0., 0., 0., 0.}, worst_r = 0.;
        int ncat[4] = {0, 0, 0, 0};
        int worst_i = -1;
        for (int i = 0; i < nS; ++i) {
            const double I = eb.value(tet_of(&Ts[size_t(12 * i)]), 0.);
            if (worst_i < 0 || rel(I, Is[size_t(i)]) > rel(eb.value(tet_of(&Ts[size_t(12 * worst_i)]), 0.), Is[size_t(worst_i)])) worst_i = i;
            worst_cat[cat[size_t(i)]] = std::max(worst_cat[cat[size_t(i)]], rel(I, Is[size_t(i)]));
            ++ncat[cat[size_t(i)]];
        }
        std::printf("   worst sample cell %d (%s)\n", worst_i, cats[cat[size_t(worst_i)]]);
        for (int i = 0; i < nR; ++i) {
            ExactBandIntegral::Stats st;
            const double I = eb.value(tet_of(&Tr[size_t(12 * i)]), 0., &st);
            worst_r = std::max(worst_r, rel(I, Ir[size_t(i)]));
            if (rel(I, Ir[size_t(i)]) > 3e-13) {
                std::printf(
                    "   random cell %d: %.17g vs the Python %.17g (rel %.1e), rule-(a) volume %.3g of %.3g\n",
                    i,
                    I,
                    Ir[size_t(i)],
                    rel(I, Ir[size_t(i)]),
                    st.rule_a_vol,
                    st.vol);
            }
        }
        std::printf("   parity, max relative error vs the Python:");
        for (int c = 0; c < 4; ++c) {
            if (ncat[c]) std::printf("  %s (%d) %.1e", cats[c], ncat[c], worst_cat[c]);
        }
        std::printf("  | %d random band cells %.1e\n", nR, worst_r);

        // derivatives: parity with exact_mesh.value_grad_hess (every sample cell at its 4 corners,
        // the first random cells at corner i % 4), and the time of one value_grad_hess() call
        {
            std::ifstream dn(std::string(dir) + "/" + name + "_derivs.bin", std::ios::binary);
            REQUIRE(dn.good());
            int32_t dh[2];
            dn.read(reinterpret_cast<char*>(dh), sizeof(dh));
            std::vector<double> ref(size_t(13 * (4 * dh[0] + dh[1])));
            dn.read(reinterpret_cast<char*>(ref.data()), std::streamsize(ref.size() * sizeof(double)));
            REQUIRE(dn.good());
            std::array<double, 3> dworst_s = {0., 0., 0.}, dworst_r = {0., 0., 0.};
            std::vector<double> us;
            int nslow = 0;
            const auto check = [&](const Tet& Tc, const int kv, const double* r, std::array<double, 3>& w, const int id) {
                Vec3 G;
                Eigen::Matrix3d H;
                const auto a0 = clk::now();
                const double J = eb.value_grad_hess(Tc, kv, delta, G, H);
                us.push_back(1e6 * std::chrono::duration<double>(clk::now() - a0).count());
                const double vol = std::abs((Tc[1] - Tc[0]).dot((Tc[2] - Tc[0]).cross(Tc[3] - Tc[0]))) / 6.;
                const Eigen::Map<const Eigen::Vector3d> Gr(r + 1);
                const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Hr(r + 4);
                const std::array<double, 3> e = {
                    std::abs(J - r[0]) / (std::abs(r[0]) + delta * vol),
                    (G - Gr).norm() / std::max(Gr.norm(), 1e-300),
                    (H - Eigen::Matrix3d(Hr)).norm() / std::max(Eigen::Matrix3d(Hr).norm(), 1e-300)};
                for (int k = 0; k < 3; ++k) w[size_t(k)] = std::max(w[size_t(k)], e[size_t(k)]);
                if (e[2] > 1e-9) {
                    ++nslow;
                    std::printf("   derivative outlier: cell %d corner %d: J %.1e G %.1e H %.1e\n", id, kv, e[0], e[1], e[2]);
                }
            };
            for (int i = 0; i < dh[0]; ++i) {
                for (int kv = 0; kv < 4; ++kv) check(tet_of(&Ts[size_t(12 * i)]), kv, &ref[size_t(13 * (4 * i + kv))], dworst_s, i);
            }
            for (int i = 0; i < dh[1]; ++i) {
                check(tet_of(&Tr[size_t(12 * i)]), i % 4, &ref[size_t(13 * (4 * dh[0] + i))], dworst_r, -1 - i);
            }
            const auto pct = [](std::vector<double> v, double p) {
                std::sort(v.begin(), v.end());
                return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
            };
            double mean = 0.;
            for (const double u : us) mean += u / double(us.size());
            std::printf(
                "   derivatives vs the Python: %d sample cells x 4 corners: J %.1e G %.1e H %.1e | %d random"
                " cells: J %.1e G %.1e H %.1e | H over 1e-9: %d\n"
                "   value_grad_hess us per call: mean %.2f p50 %.2f p90 %.2f p99 %.2f max %.2f\n",
                dh[0], dworst_s[0], dworst_s[1], dworst_s[2], dh[1], dworst_r[0], dworst_r[1], dworst_r[2],
                nslow, mean, pct(us, 0.5), pct(us, 0.9), pct(us, 0.99), pct(us, 1.));
        }

        // time per cell, untimed calls: each cell 5 times after a warm-up, the per-cell mean
        for (const bool sample : {true, false}) {
            const int n = sample ? nS : nR;
            const double* Tp = sample ? Ts.data() : Tr.data();
            std::vector<Tet> cells(static_cast<size_t>(n));
            for (int i = 0; i < n; ++i) cells[size_t(i)] = tet_of(Tp + 12 * i);
            double sink = 0.;
            for (const Tet& T : cells) sink += eb.value(T, delta);
            std::vector<double> us(size_t(n), 0.), cb(size_t(n), 0.);
            constexpr int REP = 5;
            for (int r = 0; r < REP; ++r) {
                for (int i = 0; i < n; ++i) {
                    const auto a = clk::now();
                    sink += eb.value(cells[size_t(i)], delta);
                    us[size_t(i)] += 1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
                }
            }
            // the corner bound's search on the same cells (InputTriangles::min_mean_distance)
            for (int r = 0; r < REP; ++r) {
                for (int i = 0; i < n; ++i) {
                    int64_t best;
                    const auto a = clk::now();
                    sink += tris->min_mean_distance(cells[size_t(i)].data(), 4, best);
                    cb[size_t(i)] += 1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
                }
            }
            ExactBandIntegral::Stats st;
            st.timing = true;
            for (const Tet& T : cells) sink += eb.value(T, delta, &st);
            const auto pct = [](std::vector<double> v, double p) {
                std::sort(v.begin(), v.end());
                return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
            };
            double mean = 0., mcb = 0.;
            for (int i = 0; i < n; ++i) mean += us[size_t(i)] / n, mcb += cb[size_t(i)] / n;
            std::printf(
                "   %-14s %5d cells: us per cell mean %.2f  p50 %.2f  p90 %.2f  p99 %.2f  max %.2f"
                "   | corner bound (min_mean_distance) mean %.2f p50 %.2f p99 %.2f  [%g]\n",
                sample ? "stored sample" : "random band",
                n,
                mean,
                pct(us, 0.5),
                pct(us, 0.9),
                pct(us, 0.99),
                pct(us, 1.),
                mcb,
                pct(cb, 0.5),
                pct(cb, 0.99),
                sink == 0. ? 0. : 1.);
            const double tt = st.t_total;
            std::printf(
                "     split (timed run, %.2f us per cell with clocks): candidates %.0f%%  clipping %.0f%%"
                "  closed-form faces %.0f%%  series edges %.0f%%  other %.0f%%\n",
                1e6 * tt / n,
                100 * st.t_candidates / tt,
                100 * st.t_clip / tt,
                100 * (st.t_closed - st.t_series) / tt,
                100 * st.t_series / tt,
                100 * (tt - st.t_candidates - st.t_clip - st.t_closed) / tt);
            const double c = double(st.cells);
            std::printf(
                "     per cell: candidates %.1f  pieces %.2f  clips %.1f  point faces %.1f  line faces"
                " radial %.2f parallel %.2f series %.2f  closed edges %.1f  series edges short %.2f long %.2f"
                "  rule-(a) volume %.2g%%\n",
                st.candidates / c,
                st.pieces / c,
                st.clips / c,
                st.point_faces / c,
                st.line_faces_radial / c,
                st.line_faces_parallel / c,
                st.line_faces_series / c,
                st.closed_edges / c,
                st.series_short / c,
                st.series_long / c,
                100 * st.rule_a_vol / st.vol);
        }
    }
}
