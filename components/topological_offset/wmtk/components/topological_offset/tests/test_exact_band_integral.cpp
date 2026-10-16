#include <wmtk/components/topological_offset/ExactBandIntegral.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <catch2/catch_test_macros.hpp>

#include "exact_band_integral_data.hpp"
#include "exact_band_max_data.hpp"
#include "exact_band_parallel_data.hpp"
#include "exact_band_side_data.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
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

/// The input envelope of the triangles (V, F), as TopoOffsetTetMesh::m_input_complex_envelope is
/// to the band term's searches (OffsetPotential.hpp). eps 0 builds no containment structure,
/// which no search reads.
std::shared_ptr<const wmtk::Envelope> triangle_envelope(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    auto env = std::make_shared<wmtk::Envelope>(true);
    env->init(V, F, 0.);
    return env;
}

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
    for (int i = 0; i < 4; ++i)
        k[size_t(4 + i)] = {a, b, ring[size_t(i)], ring[size_t((i + 1) % 4)]};
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
    int64_t n_closed = 0;
    for (const auto& c : exact_band_data::tri_cells) {
        const ExactBandIntegral eb = one_triangle(c.P);
        const Tet T = tet_of(c.T);
        ExactBandIntegral::Stats st;
        st.check_closed = true; // every clip result a closed surface (clip())
        const double I = eb.value(T, 0., &st);
        INFO(c.label);
        CHECK(st.open_results == 0);
        n_closed += st.closed_checked;
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
    std::printf(
        "exact-band-integral-one-triangle: max relative error per configuration (%lld clip results"
        " checked closed)\n",
        (long long)n_closed);
    std::printf("  %-30s %9s %9s %9s %7s\n", "", "vs mesh", "vs tri", "additiv.", "series");
    for (const auto& [k, w] : worst) {
        std::printf("  %-30s %9.1e %9.1e %9.1e %7.0f\n", k.c_str(), w[0], w[1], w[2], w[3]);
    }
}

TEST_CASE("exact-band-integral-real-cells", "[offset][exact_band]")
{
    std::printf("exact-band-integral-real-cells: max relative error against the Python\n");
    int64_t n_closed = 0;
    for (const auto& c : exact_band_data::real_cases) {
        const ExactBandIntegral eb = of_case(c);
        double worst = 0., worst_add = 0.;
        int n_add = 0, n_rule_a = 0;
        std::vector<double> J(size_t(c.ncells));
        for (int i = 0; i < c.ncells; ++i) {
            const auto& cell = c.cells[i];
            const Tet T = tet_of(cell.T);
            ExactBandIntegral::Stats st;
            st.check_closed = true; // every clip result a closed surface (clip())
            const double I = eb.value(T, 0., &st);
            J[size_t(i)] = eb.value(T, c.delta);
            INFO(c.name << " cell " << i << " (" << cell.cat << ")");
            CHECK(st.open_results == 0);
            n_closed += st.closed_checked;
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
                sk.check_closed = true;
                sum += eb.value(k, 0., &sk);
                exact = exact && sk.rule_a_vol == 0.;
                CHECK(sk.open_results == 0);
                n_closed += sk.closed_checked;
            }
            if (!exact) continue;
            CHECK(rel(sum, I) <= 1e-12);
            worst_add = std::max(worst_add, rel(sum, I));
            ++n_add;
        }
        std::printf(
            "  %-13s %2d cells (%d with rule (a)): max %.1e; additivity on %d exact families: max "
            "%.1e\n",
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
    std::printf("  every clip result checked closed: %lld\n", (long long)n_closed);
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
        {Vec3(0.1, 0.1, 0),
         Vec3(0.9, 0.05, 0),
         Vec3(0.3, 0.7, 0),
         Vec3(0.4, 0.3, 0)}, // on P's plane
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
        rel_vec(
            Eigen::Map<const Eigen::VectorXd>(H.data(), 9),
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
            worst[0],
            worst[1],
            worst[2],
            worst_far[0],
            worst_far[1],
            worst_far[2]);
        std::printf(
            "  one triangle, all corners: %d smooth: FD min over h, max: gradient %.1e Hessian "
            "%.1e; "
            "%d kink corners: inward one-sided (h=1e-6) max %.1e | far away, %d corners: %.1e "
            "%.1e\n",
            fs.smooth,
            fs.grad,
            fs.hess,
            fs.kinks,
            fs.inward,
            fs_far.smooth,
            fs_far.grad,
            fs_far.hess);
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
            c.name,
            c.nderivs,
            worst[0],
            worst[1],
            worst[2],
            fs.smooth,
            fs.grad,
            fs.hess,
            fs.kinks,
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
    const auto tris = triangle_envelope(V, F);
    const double pad = input_pad(V);
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
    BandVolumeEnergy3D energy(tris, pad, cells, ctr, delta);
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
        std::vector<double> Vb(static_cast<size_t>(3 * nV)), Ts(static_cast<size_t>(12 * nS)),
            Is(static_cast<size_t>(nS)), Tr(static_cast<size_t>(12 * nR)),
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
        for (int i = 0; i < nV; ++i)
            V.row(i) << Vb[size_t(3 * i)], Vb[size_t(3 * i + 1)], Vb[size_t(3 * i + 2)];
        for (int i = 0; i < nF; ++i)
            F.row(i) << Fb[size_t(3 * i)], Fb[size_t(3 * i + 1)], Fb[size_t(3 * i + 2)];
        auto t0 = clk::now();
        const ExactBandIntegral eb(V, F);
        const double t_build = std::chrono::duration<double>(clk::now() - t0).count();
        const auto tris = triangle_envelope(V, F);
        const double pad = input_pad(V);

        std::printf(
            "== %s: %d triangles (build %.2f ms), delta %.6g\n",
            name,
            nF,
            1e3 * t_build,
            delta);
        // parity
        double worst_cat[4] = {0., 0., 0., 0.}, worst_r = 0.;
        int ncat[4] = {0, 0, 0, 0};
        int worst_i = -1;
        for (int i = 0; i < nS; ++i) {
            const double I = eb.value(tet_of(&Ts[size_t(12 * i)]), 0.);
            if (worst_i < 0 ||
                rel(I, Is[size_t(i)]) >
                    rel(eb.value(tet_of(&Ts[size_t(12 * worst_i)]), 0.), Is[size_t(worst_i)]))
                worst_i = i;
            worst_cat[cat[size_t(i)]] = std::max(worst_cat[cat[size_t(i)]], rel(I, Is[size_t(i)]));
            ++ncat[cat[size_t(i)]];
        }
        std::printf("   worst sample cell %d (%s)\n", worst_i, cats[cat[size_t(worst_i)]]);
        int64_t n_checked = 0, n_open = 0;
        for (int i = 0; i < nR; ++i) {
            ExactBandIntegral::Stats st;
            st.check_closed = true;
            const double I = eb.value(tet_of(&Tr[size_t(12 * i)]), 0., &st);
            n_checked += st.closed_checked;
            n_open += st.open_results;
            worst_r = std::max(worst_r, rel(I, Ir[size_t(i)]));
            if (rel(I, Ir[size_t(i)]) > 3e-13) {
                std::printf(
                    "   random cell %d: %.17g vs the Python %.17g (rel %.1e), rule-(a) volume %.3g "
                    "of %.3g\n",
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
        std::printf(
            "  | %d random band cells %.1e | clip results checked closed %lld, open %lld\n",
            nR,
            worst_r,
            (long long)n_checked,
            (long long)n_open);
        CHECK(n_open == 0);

        // derivatives: parity with exact_mesh.value_grad_hess (every sample cell at its 4 corners,
        // the first random cells at corner i % 4), and the time of one value_grad_hess() call
        {
            std::ifstream dn(std::string(dir) + "/" + name + "_derivs.bin", std::ios::binary);
            REQUIRE(dn.good());
            int32_t dh[2];
            dn.read(reinterpret_cast<char*>(dh), sizeof(dh));
            std::vector<double> ref(size_t(13 * (4 * dh[0] + dh[1])));
            dn.read(
                reinterpret_cast<char*>(ref.data()),
                std::streamsize(ref.size() * sizeof(double)));
            REQUIRE(dn.good());
            std::array<double, 3> dworst_s = {0., 0., 0.}, dworst_r = {0., 0., 0.};
            std::vector<double> us;
            int nslow = 0;
            const auto check = [&](const Tet& Tc,
                                   const int kv,
                                   const double* r,
                                   std::array<double, 3>& w,
                                   const int id) {
                Vec3 G;
                Eigen::Matrix3d H;
                const auto a0 = clk::now();
                const double J = eb.value_grad_hess(Tc, kv, delta, G, H);
                us.push_back(1e6 * std::chrono::duration<double>(clk::now() - a0).count());
                const double vol =
                    std::abs((Tc[1] - Tc[0]).dot((Tc[2] - Tc[0]).cross(Tc[3] - Tc[0]))) / 6.;
                const Eigen::Map<const Eigen::Vector3d> Gr(r + 1);
                const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Hr(r + 4);
                const std::array<double, 3> e = {
                    std::abs(J - r[0]) / (std::abs(r[0]) + delta * vol),
                    (G - Gr).norm() / std::max(Gr.norm(), 1e-300),
                    (H - Eigen::Matrix3d(Hr)).norm() /
                        std::max(Eigen::Matrix3d(Hr).norm(), 1e-300)};
                for (int k = 0; k < 3; ++k) w[size_t(k)] = std::max(w[size_t(k)], e[size_t(k)]);
                if (e[2] > 1e-9) {
                    ++nslow;
                    std::printf(
                        "   derivative outlier: cell %d corner %d: J %.1e G %.1e H %.1e\n",
                        id,
                        kv,
                        e[0],
                        e[1],
                        e[2]);
                }
            };
            for (int i = 0; i < dh[0]; ++i) {
                for (int kv = 0; kv < 4; ++kv)
                    check(
                        tet_of(&Ts[size_t(12 * i)]),
                        kv,
                        &ref[size_t(13 * (4 * i + kv))],
                        dworst_s,
                        i);
            }
            for (int i = 0; i < dh[1]; ++i) {
                check(
                    tet_of(&Tr[size_t(12 * i)]),
                    i % 4,
                    &ref[size_t(13 * (4 * dh[0] + i))],
                    dworst_r,
                    -1 - i);
            }
            const auto pct = [](std::vector<double> v, double p) {
                std::sort(v.begin(), v.end());
                return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
            };
            double mean = 0.;
            for (const double u : us) mean += u / double(us.size());
            std::printf(
                "   derivatives vs the Python: %d sample cells x 4 corners: J %.1e G %.1e H %.1e | "
                "%d random"
                " cells: J %.1e G %.1e H %.1e | H over 1e-9: %d\n"
                "   value_grad_hess us per call: mean %.2f p50 %.2f p90 %.2f p99 %.2f max %.2f\n",
                dh[0],
                dworst_s[0],
                dworst_s[1],
                dworst_s[2],
                dh[1],
                dworst_r[0],
                dworst_r[1],
                dworst_r[2],
                nslow,
                mean,
                pct(us, 0.5),
                pct(us, 0.9),
                pct(us, 0.99),
                pct(us, 1.));
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
                    us[size_t(i)] +=
                        1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
                }
            }
            // the corner bound's search on the same cells (min_mean_distance())
            for (int r = 0; r < REP; ++r) {
                for (int i = 0; i < n; ++i) {
                    int64_t best;
                    const auto a = clk::now();
                    sink += min_mean_distance(*tris, pad, cells[size_t(i)].data(), 4, best);
                    cb[size_t(i)] +=
                        1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
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
                "     split (timed run, %.2f us per cell with clocks): candidates %.0f%%  clipping "
                "%.0f%%"
                "  closed-form faces %.0f%%  series edges %.0f%%  other %.0f%%\n",
                1e6 * tt / n,
                100 * st.t_candidates / tt,
                100 * st.t_clip / tt,
                100 * (st.t_closed - st.t_series) / tt,
                100 * st.t_series / tt,
                100 * (tt - st.t_candidates - st.t_clip - st.t_closed) / tt);
            const double c = double(st.cells);
            std::printf(
                "     per cell: candidates %.1f  pieces %.2f  clips %.1f  point faces %.1f  line "
                "faces"
                " radial %.2f parallel %.2f series %.2f  closed edges %.1f  series edges short "
                "%.2f long %.2f"
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

// Data of exact-band-integral-geneva: GENERATED by gen_geneva_data.py (scratchpad exact_port/,
// 2026-10-12) from the Python reference exact_integral/exact_mesh.py. Hex floats: bit exact.
// Generated table, kept as the generator writes it: not reformatted.
// clang-format off
namespace exact_band_geneva_data {

/// A cell T, delta, and I = exact_mesh.integrate_mesh(T) = int_T d dV on the full triangle set
/// (equal, bit for bit, on the subset below).
struct GCell { double T[12]; double delta, I; };
inline const double geneva_run_V[] = {-0x1.e4c1244ccd2c1p+3, -0x1.0f6a88739f539p+2, -0x1.0a1cb86ac17bfp-1, -0x1.d78d71fa34b23p+3, -0x1.70f2263653dd3p+2, -0x1.0a1ca7ac7886dp-1, -0x1.d17409f34290ap+3, -0x1.368e041a3b1efp+2, -0x1.0a1ca58686bbfp-1, -0x1.cd18a218a4fcep+3, -0x1.c338f2573bdb6p+1, -0x1.0a1caab335810p-1, -0x1.c0b7fc3bbdddbp+3, -0x1.125358b48c8cfp+2, -0x1.0a1cbf6b69176p-1, -0x1.bf1f138c7c3bbp+3, -0x1.59e3d785e5684p+2, -0x1.0a1ca0fdab580p-1, -0x1.b1fa9c6df4f1dp+3, -0x1.e29e3da0f85c3p+1, -0x1.0a1caa1b9091ap-1, -0x1.ae34450120391p+3, -0x1.41c46ea97b73cp+2, -0x1.0a1cb163a1198p-1, -0x1.a5ea46998011ep+3, -0x1.84d4019978413p+2, -0x1.0a1cbda1275ecp-1, -0x1.9c7464029f71cp+3, -0x1.1fa9abf2367c2p+2, -0x1.0a1cc1922842dp-1, -0x1.9c2f18a702445p+3, -0x1.4ce2c2954a3fdp+2, -0x1.0a1c8819ffa7ap-1, -0x1.9b81343737dbcp+3, -0x1.e1da5475b356ap+1, -0x1.0a1cb0eeec916p-1, -0x1.833a94ff28c2bp+3, -0x1.04a72732dea2bp+2, -0x1.0a1cccba52480p-1, -0x1.825c678e36b1ep+3, -0x1.4e813a98ea216p+2, -0x1.0a1caab4fa03fp-1, -0x1.822c739be93bbp+3, 0x1.72c16c27607b0p+1, -0x1.0a1cc91218161p-1, -0x1.77d28b583c9b9p+3, 0x1.0eceb2516dc40p+1, -0x1.0a1caba5ea276p-1, -0x1.6dc036b35e43cp+3, 0x1.709c07683b4b5p+0, -0x1.0a1c99b13add7p-1, -0x1.627e1ff0682eap+3, 0x1.1ec9bcce9f8e1p+1, -0x1.0a1cc65ff866cp-1, -0x1.5abf816cbcea0p+3, 0x1.7673efcc2aef3p+1, -0x1.0a1cb54f93ff4p-1, -0x1.51c809c94406fp+3, 0x1.d8e20b27b59cdp-1, -0x1.0a1ccc46fab5cp-1, -0x1.47d3ba3d293c5p+3, 0x1.cc17c69d77dfbp+1, -0x1.0a1caa3c73073p-1, -0x1.43b40c0372d2ap+3, 0x1.bcf9eca078f6cp+0, -0x1.0a1cac33b6b75p-1, -0x1.3fab6445835f4p+3, 0x1.5e3bc9dfd0ac0p+1, -0x1.0a1caa19a3c00p-1, -0x1.3a4e690a222dfp+3, 0x1.abf36a16843ecp-1, -0x1.0a1cb203e6b6bp-1, -0x1.36e79304442a1p+3, 0x1.abb6db38ed71cp+1, -0x1.0a1c9e995eb9fp-1, -0x1.3171dd17a04bap+3, 0x1.1d3d5b618dd64p+1, -0x1.0a1cb29158c66p-1, -0x1.290bd13e71066p+3, 0x1.448ad4bbd571bp+0, -0x1.0a1cae5276385p-1, -0x1.25815cab3f48dp+3, 0x1.8b03604f694acp+1, -0x1.0a1cd0b91a15bp-1, -0x1.83f931d176605p+2, -0x1.31791de0b97dcp+0, 0x1.d80fcd79ce4c5p-4, -0x1.8292ec0c066b8p+2, -0x1.17dc2242e9241p+1, -0x1.0a1cbf42b2d9ep-1, -0x1.7a917d6b65a9bp+2, -0x1.2c4840e1719f8p+1, 0x1.d810624dd2f1bp-4, -0x1.6a848e726a0e4p+2, -0x1.a0895e8337d54p+1, -0x1.0a1c9bf7152fep-1, -0x1.51f2720fae803p+2, -0x1.7823a3a4530e3p+0, -0x1.0a1cb45dc1a07p-1, -0x1.4f5ea0f416e1cp+2, -0x1.907db81b8ce7dp+1, 0x1.d8102a66e9182p-4, -0x1.410908d56a864p+2, -0x1.f62bd31601b68p+0, 0x1.d810791bb2c33p-4, -0x1.3c46a4455ec77p+2, -0x1.aeffe66b8076fp+1, -0x1.0a1ca6bda1ba1p-1, -0x1.37eff62674fa5p+2, -0x1.42d77efb95b59p+1, -0x1.0a1cc4a53e97fp-1, -0x1.258910ea127b9p+2, -0x1.5756af27c02c1p+1, 0x1.d810624dd2f1bp-4, -0x1.240f273a97d7ap+2, -0x1.6423de7402370p+0, 0x1.d81026e48e5f5p-4, -0x1.20f6f2b951eccp+2, -0x1.d7685421229fcp-1, -0x1.0a1cac61f691dp-1, -0x1.1cf9557404d98p+2, -0x1.bf02594149934p+0, -0x1.0a1ca0d061ac8p-1, -0x1.17902b9101c8dp+2, -0x1.0439382f1f944p+1, 0x1.d81064af078e9p-4, -0x1.095136789933ep+2, 0x1.961fb3bbc6152p-1, -0x1.0a1c9ee2d2378p-1, -0x1.07cada12f8579p+2, -0x1.ae0f88bac5d4dp+1, -0x1.0a1caa70a2430p-1, -0x1.007575ac69d2fp+2, -0x1.ef39c94a03976p-1, 0x1.d810cca59d5a4p-4, -0x1.ec6e202e3eb0bp+1, -0x1.543907920963dp-1, -0x1.0a1c97d7ad297p-1, -0x1.e755fbff233eep+1, -0x1.32b19b21cec2cp+1, 0x1.d81062907726cp-4, -0x1.e73f9d1c5bd4cp+1, 0x1.06ddc7dcfaf0bp+2, 0x1.d810722fb4da0p-4, -0x1.e048c976d0efap+1, -0x1.4361cabef7555p+1, -0x1.0a1c9e7fc85d1p-1, -0x1.dab1c7fda7011p+1, -0x1.80d231af002e9p+0, -0x1.0a1cb8570f836p-1, -0x1.d474162414dbdp+1, -0x1.ca9e05e51806dp+0, 0x1.d810723326c52p-4, -0x1.ce011da762d4cp+1, 0x1.d2df529cde26bp-6, -0x1.0a1ccae402d72p-1, -0x1.b97eb00659e22p+1, 0x1.796ecfcfb04dap+2, 0x1.d810a5c981005p-4, -0x1.b67c2247a467bp+1, 0x1.5eae66c9a2e7dp+0, -0x1.0a1ccdd9073a9p-1, -0x1.abd42b4ce23c4p+1, 0x1.3fa8dbc98f955p+2, 0x1.d80f42491b2acp-4, -0x1.a3609404885b9p+1, -0x1.2022bec93365fp+0, 0x1.d8102b82d4c97p-4, -0x1.a10148d17e9aep+1, -0x1.82651d6e0eb8ap+1, 0x1.d810624dd2f1bp-4, -0x1.9d537ca52d629p+1, -0x1.b3166b4cf60bcp+1, -0x1.0a1cadc18aa81p-1, -0x1.92c2daa2ec3aap+1, 0x1.45218325f3275p-1, -0x1.0a1ca7cfef941p-1, -0x1.81fd9ddf08672p+1, -0x1.554e01b08319dp+1, 0x1.d817e912ff867p-4, -0x1.77d7203e911d2p+1, -0x1.d85c5865429d4p-1, -0x1.0a1cc11ad9406p-1, -0x1.75fa27d817e1bp+1, -0x1.b1ff3d2fd0bc7p+0, 0x1.d8105efeb70d7p-4, -0x1.74f37525cb0fep+1, 0x1.d1002cd903b14p+1, 0x1.d80fbfb557c8ep-4, -0x1.725397b370ce2p+1, -0x1.0510077ff2728p+1, -0x1.0a1cac0affcbcp-1, -0x1.6786f33d5ad1ap+1, 0x1.186d4b3473f71p+2, 0x1.d811bbfaae6b5p-4, -0x1.621a13056b7c1p+1, 0x1.6d3ff0f95d47ep+2, 0x1.d8102391804d9p-4, -0x1.61e1cf47a467fp+1, -0x1.6509f232d9ff6p+1, -0x1.0a1cb2e4dffdfp-1, -0x1.5dffa0f132b4ep+1, -0x1.21e54b866c179p-4, -0x1.0a1cb496a505ep-1, -0x1.4e8c789ea7e38p+1, 0x1.174b1623d6512p+0, -0x1.0a1cb4a19aa2bp-1, -0x1.4868ac157f3a9p+1, 0x1.44bc87bbfb1d6p+2, 0x1.d80ff00d10b50p-4, -0x1.17657e732abb4p+1, 0x1.075d5aa0cc48ep-1, -0x1.0a1c8bf27a07dp-1, -0x1.0f5d29204c996p+1, 0x1.e6f29f7634c20p+1, 0x1.d80fb4ecba094p-4, -0x1.f40dafe71ea88p+0, 0x1.6bf4e5a63304fp+2, 0x1.d810e0750b048p-4, -0x1.e674b31568c2ap+0, -0x1.9dcfc01cdd3afp-2, -0x1.0a1cc357df1b6p-1, -0x1.bc0a7bf1a7a1ap+0, 0x1.2607d614aed9bp+2, 0x1.d8115938dc770p-4, -0x1.b6e97924d2bd9p+0, 0x1.3dcccc18e8249p+0, -0x1.0a1cbc16e7b11p-1, -0x1.7e4efa5c9b0ddp+0, 0x1.2ce42798dfdacp-2, -0x1.0a1ca2e96ab1bp-1, -0x1.33a0e3b2d5281p+0, 0x1.5cfc635957721p+2, 0x1.d80fde74b0d65p-4, -0x1.1c10cc2cc2261p+0, 0x1.144f6b5e83f03p+2, 0x1.d81151ed1cea4p-4, -0x1.dd6fa0987eec1p-1, 0x1.35611f60e2132p+2, 0x1.d8111aee6bbbdp-4, -0x1.7b73077f2322bp-1, 0x1.b41fcc3923657p+2, -0x1.0a1cb0145b230p-1, -0x1.469a93d33dafap-1, 0x1.efb7c08d34780p+2, -0x1.0a1ccf1fae90ep-1, -0x1.1577bcd033223p-3, -0x1.bdbf7b5b11baap+1, -0x1.0a1cc34795565p-1, -0x1.b0def8bd83302p-4, -0x1.5d331c5a79a8bp+1, -0x1.0a1c8eb591a3cp-1, 0x1.0274db6aa10d0p-7, -0x1.36fec793b7abap+2, -0x1.0a1cba9ad1779p-1, 0x1.99bb21f978957p-5, 0x1.9386fd1cc4f20p+2, -0x1.0a1ca8cb371f7p-1, 0x1.20bf61feb31cdp-3, 0x1.02e56d65b83f3p+3, -0x1.0a1cb7b4b988ep-1, 0x1.73067f3983d91p-3, -0x1.041e9258de0adp+2, -0x1.0a1ca46130725p-1, 0x1.6acc7b94493b3p-2, 0x1.c4b539bb447f3p+2, -0x1.0a1caa3cf8d5dp-1, 0x1.2fd657622b9bfp-1, -0x1.88b318a08272fp+1, -0x1.0a1cbc3f19d9ep-1, 0x1.300e05eb74e4cp-1, -0x1.33120317c8fdap+1, -0x1.0a1c93e07f89ap-1, 0x1.90bc0173a87ecp-1, 0x1.ef7fe8956bd64p+2, -0x1.0a1c8f0db2dbfp-1, 0x1.a4ac627dd952ep-1, -0x1.02ead6cbe769cp+2, -0x1.0a1cb9eb1efbbp-1, 0x1.e61c41262893ap-1, -0x1.5467ac5d006e9p+2, -0x1.0a1cbdf1c0101p-1, 0x1.3f9a69abe2260p+0, -0x1.38ca3e2171ab6p+1, -0x1.0a1ca4d890cf7p-1, 0x1.417ebb5d382c6p+0, 0x1.908c553d674b2p+2, -0x1.0a1ccc1fae6fdp-1, 0x1.42ecba3cd6420p+0, 0x1.0e33e69e219c4p+3, -0x1.0a1cac083126fp-1, 0x1.64fd01ca42ddcp+0, -0x1.d24b272e63758p+1, -0x1.0a1cb60d4ca43p-1, 0x1.6dd777a2f9c4cp+0, 0x1.d9c48cd8ff2e7p+2, -0x1.0a1cb18bf5a8bp-1, 0x1.90fed94df03b4p+0, -0x1.25ec1e48d5a56p+2, -0x1.0a1ca0fa92895p-1, 0x1.cc2b1c63a43f0p+0, 0x1.0080e72104d04p+3, -0x1.0a1c99d6f49d9p-1, 0x1.e0bd08b0977e2p+0, -0x1.6975dae627530p+1, -0x1.0a1cbc89e0515p-1, 0x1.029fb6a717ea6p+1, 0x1.b95972d20f258p+2, -0x1.0a1cb55a6cb1cp-1, 0x1.2d9b6b7cc0866p+1, -0x1.d9e74deb34c75p+1, -0x1.0a1cb753997e7p-1, 0x1.2dadefa30317bp+1, 0x1.ef89ae023be48p+2, -0x1.0a1cbf8f79804p-1, 0x1.2db5847685f0dp+2, 0x1.7013e7274d8c0p+2, 0x1.d8102bfe9c44ap-4, 0x1.3c7a6544041c4p+2, 0x1.a292e8f8ac6cbp+2, 0x1.d8107bac046ccp-4, 0x1.4dbe86e615311p+2, 0x1.52a362a34a61cp+2, 0x1.d80ff5fe5ca29p-4, 0x1.5571aae8f9b18p+2, 0x1.e0f895366ec01p+2, 0x1.d810d251c9d22p-4, 0x1.6c36bef58afd4p+2, 0x1.adace121285adp+2, 0x1.d81016f59dbcdp-4, 0x1.73bde99ee84d1p+2, 0x1.2aa85546d8746p+2, 0x1.d80fbd6c30f9ep-4, 0x1.770aa1ddf89dfp+2, 0x1.710b984ba91b6p+2, 0x1.d810d5a01c86bp-4, 0x1.81b5fecce2700p+2, 0x1.db36f2880a1cfp+2, 0x1.d810bf82df3dep-4, 0x1.a1e93533acba1p+2, 0x1.0286cfb8c5a33p+3, 0x1.d810678ae8870p-4, 0x1.a5fcee419d4c2p+2, 0x1.bc0f7383d26e5p+2, 0x1.d810b27e90355p-4, 0x1.ab5c8fa5049d6p+2, 0x1.3ba7bc8a6ce4ep+2, 0x1.d811bfa0be671p-4, 0x1.bec27a5f3e26ap+2, 0x1.80b946ef79433p+2, 0x1.d8106411fb15dp-4, 0x1.c3144c193b5bap+2, 0x1.e94344c4403afp+2, 0x1.d8116015bbbd0p-4, 0x1.ddee57f81f189p+2, 0x1.5689f4cbf88b1p+2, 0x1.d810fc9ee7023p-4, 0x1.f448ed5cf8338p+2, 0x1.c19b48b6371dbp+2, 0x1.d80fcfa20ba25p-4};
inline const int geneva_run_F[] = {0, 3, 4, 0, 4, 2, 1, 2, 5, 8, 5, 7, 8, 7, 10, 26, 21, 25, 26, 21, 23, 29, 31, 36, 29, 32, 36, 30, 37, 33, 30, 37, 34, 28, 38, 34, 81, 80, 88, 81, 88, 86, 96, 98, 91, 96, 98, 100, 96, 86, 91, 94, 101, 89, 94, 89, 90, 101, 89, 97, 101, 97, 103, 114, 111, 116, 114, 111, 109, 114, 113, 117, 114, 113, 112, 114, 119, 117, 114, 119, 116, 114, 109, 112, 59, 56, 46, 59, 61, 50, 59, 50, 46, 56, 37, 46, 73, 76, 70, 73, 60, 67, 73, 70, 67, 76, 75, 70, 68, 75, 70, 68, 53, 58, 68, 58, 70, 51, 60, 45, 51, 60, 67, 51, 58, 67, 51, 58, 42, 78, 71, 74, 78, 79, 74, 71, 62, 64, 71, 74, 64, 62, 47, 64, 79, 74, 77, 74, 72, 69, 74, 72, 77, 74, 64, 69, 87, 84, 92, 87, 92, 89, 87, 82, 89, 84, 92, 93, 92, 93, 99, 92, 89, 97, 92, 97, 99, 95, 85, 88, 95, 88, 98, 95, 98, 102, 115, 111, 116, 115, 111, 110, 115, 118, 116, 3, 4, 6, 4, 6, 9, 4, 2, 5, 4, 5, 7, 4, 9, 7, 11, 6, 9, 11, 9, 12, 9, 12, 13, 9, 13, 10, 9, 7, 10, 15, 14, 17, 15, 17, 16, 14, 18, 17, 19, 21, 16, 19, 21, 23, 21, 25, 22, 21, 17, 22, 21, 17, 16, 20, 18, 22, 20, 24, 22, 25, 22, 27, 31, 36, 35, 32, 36, 40, 32, 39, 40, 36, 43, 48, 36, 43, 35, 36, 48, 40, 47, 54, 64, 53, 58, 42, 82, 89, 83, 89, 90, 83, 111, 106, 105, 111, 106, 109, 111, 110, 107, 111, 105, 107, 66, 48, 57, 66, 48, 63, 49, 39, 40, 49, 48, 40, 49, 48, 63, 85, 80, 88, 38, 41, 50, 38, 41, 34, 38, 50, 44, 106, 108, 109, 54, 52, 65, 54, 65, 69, 54, 64, 69, 18, 17, 22, 104, 98, 102, 104, 98, 100, 108, 109, 112, 72, 65, 69, 97, 103, 99, 88, 98, 91, 88, 86, 91, 37, 41, 46, 37, 41, 34, 24, 22, 27, 58, 70, 67, 41, 50, 46, 43, 48, 57, 61, 50, 55, 50, 44, 55};
inline const GCell geneva_run_cells[] = {
    {{0x1.6dd777a2f9c4cp+0, 0x1.d9c48cd8ff2e7p+2, -0x1.0a1cb18bf5a8bp-1, 0x1.cc90e253d25ccp-2, 0x1.d5c0fbe20f9b4p+2, -0x1.fa5f7e554dbe1p-2, 0x1.809d965abf494p-1, 0x1.f0e9e9269375ep+2, -0x1.0a1c915b519dbp-1, 0x1.6acc7b94493b3p-2, 0x1.c4b539bb447f3p+2, -0x1.0a1caa3cf8d5dp-1}, 0x1.fca85caafbc1bp-6, 0x1.10898624c7d08p-16},
    {{0x1.a51f13a20fc5cp+2, 0x1.a4dfc68c46285p+2, 0x1.6f2e28b364dd7p-4, 0x1.6c00dd4360d6ep+2, 0x1.adebfe42d5a43p+2, 0x1.d810182185ddbp-4, 0x1.770aa1ddf89dfp+2, 0x1.710b984ba91b6p+2, 0x1.d810d5a01c86bp-4, 0x1.a5fcee419d4c2p+2, 0x1.bc0f7383d26e5p+2, 0x1.d810b27e90355p-4}, 0x1.fca85caafbc1bp-6, 0x1.9c7d2fb422c24p-16},
    {{-0x1.9c7464029f71cp+3, -0x1.1fa9abf2367c2p+2, -0x1.0a1cc1922842dp-1, -0x1.bf1cc6d518975p+3, -0x1.18bc31f701058p+2, -0x1.025fc927f3992p-1, -0x1.c0b7fc3bbdddbp+3, -0x1.125358b48c8cfp+2, -0x1.0a1cbf6b69176p-1, -0x1.ae4916391ba19p+3, -0x1.4253a460553acp+2, -0x1.0a1cb15580d8cp-1}, 0x1.fca85caafbc1bp-6, 0x1.d232b176dc5d4p-18},
    {{-0x1.164cb99894edap+2, -0x1.05536455acd8dp+1, 0x1.b72ebdd8edcc5p-4, -0x1.21e1df723c73cp+2, -0x1.001d39bcfe695p+1, 0x1.d8106831e7da6p-4, -0x1.e4c59e70d8a6cp+1, -0x1.356f4c5a73fa2p+1, 0x1.d81066819a88dp-4, -0x1.d474162414dbdp+1, -0x1.ca9e05e51806dp+0, 0x1.d810723326c52p-4}, 0x1.fca85caafbc1bp-6, 0x1.7521d965896d0p-20},
    {{-0x1.439d2bc8ee8d6p+3, 0x1.b922c4d5fd071p+0, -0x1.0a1cac60cf8adp-1, -0x1.40531eabbce71p+3, 0x1.f8a4bcd42d342p+0, -0x1.fd614d3b1da0dp-2, -0x1.3fab6445835f4p+3, 0x1.5e3bc9dfd0ac0p+1, -0x1.0a1caa19a3c00p-1, -0x1.627e1ff0682eap+3, 0x1.1ec9bcce9f8e1p+1, -0x1.0a1cc65ff866cp-1}, 0x1.fca85caafbc1bp-6, 0x1.6a41fb4963e90p-16},
    {{-0x1.6786f33d5ad1ap+1, 0x1.186d4b3473f71p+2, 0x1.d811bbfaae6b5p-4, -0x1.48afa23d8bb4cp+1, 0x1.4564d4dbf25a6p+2, 0x1.d80ff12517019p-4, -0x1.bc0a7bf1a7a1ap+0, 0x1.2607d614aed9bp+2, 0x1.d8115938dc770p-4, -0x1.c385f095176c9p+0, 0x1.26c6b61fd8394p+2, 0x1.b713ae99061d2p-4}, 0x1.fca85caafbc1bp-6, 0x1.fe82309186e88p-20},
    {{-0x1.1cf9557404d98p+2, -0x1.bf02594149934p+0, -0x1.0a1ca0d061ac8p-1, -0x1.277df06a5ac42p+2, -0x1.0666f17f0b310p+1, -0x1.fcb7b87595cf7p-2, -0x1.37eff62674fa5p+2, -0x1.42d77efb95b59p+1, -0x1.0a1cc4a53e97fp-1, -0x1.dfaff5a67b5ebp+1, -0x1.43c6fcc8a2516p+1, -0x1.0a1c9e9b4fb05p-1}, 0x1.fca85caafbc1bp-6, 0x1.432af0af07855p-16},
    {{-0x1.439d2bc8ee45ep+3, 0x1.b922c4d602036p+0, -0x1.0a1cac60cf8adp-1, -0x1.40531eabbce71p+3, 0x1.f8a4bcd42d342p+0, -0x1.fd614d3b1da0dp-2, -0x1.3fab6445835f4p+3, 0x1.5e3bc9dfd0ac0p+1, -0x1.0a1caa19a3c00p-1, -0x1.627e1ff0682eap+3, 0x1.1ec9bcce9f8e1p+1, -0x1.0a1cc65ff866cp-1}, 0x1.fca85caafbc1bp-6, 0x1.6a41fb495e0afp-16},
    {{-0x1.17123343e0672p+1, 0x1.07d7561ce48bap-1, -0x1.0a1c8c18b8e2dp-1, -0x1.92c2daa2ec3aap+1, 0x1.45218325f3275p-1, -0x1.0a1ca7cfef941p-1, -0x1.5f194490c0c86p+1, -0x1.70d5c1b4d7a98p-4, -0x1.0a1cb4fa66e43p-1, -0x1.91a15f67854b8p+1, 0x1.3d8ecb01a0809p-1, -0x1.0636b1ff978e1p-1}, 0x1.fca85caafbc1bp-6, 0x1.a6f88de490a36p-20},
    {{0x1.5f097f46ec6f4p-1, -0x1.948acf7df299ep+1, -0x1.0a1cbb83eeb85p-1, 0x1.852e42efb82ebp-1, -0x1.cf6d5dde79912p+1, -0x1.f45de64569b24p-2, 0x1.87e22a3a3a84bp-1, -0x1.11ef9734a1593p+2, -0x1.0a1cba7e0959fp-1, 0x1.392890936d814p+0, -0x1.c3f0a305c8748p+1, -0x1.0a1cb75e548d1p-1}, 0x1.fca85caafbc1bp-6, 0x1.835c5baf125ecp-16},
};
inline const double geneva_tag0_V[] = {-0x1.660ccbe9695f6p+3, 0x1.a6a60150d314bp+2, -0x1.0a1cb2b071ebdp-1, -0x1.589b008ff27bbp+3, 0x1.aeb80bc3af712p+1, 0x1.d810acef3dcfcp-4, -0x1.57e205c5db0e3p+3, 0x1.7a935c37618e6p+2, 0x1.d8110f523b30dp-4, -0x1.56001e6536d65p+3, 0x1.09b49f1a2e45ap+2, 0x1.d810d311f19c0p-4, -0x1.545a03e05bc16p+3, 0x1.48923d39e4c24p+2, -0x1.0a1ccc632b0f3p-1, -0x1.4f021cf22724bp+3, 0x1.487b94c3a4437p+2, 0x1.d80ece418e5b3p-4, -0x1.4dc677b10159ap+3, 0x1.a253392156eebp+2, 0x1.d8103437ce03ap-4, -0x1.4c299bffd1107p+3, 0x1.180068aac8740p+2, -0x1.0a1cbaf990250p-1, -0x1.49d9c645142b6p+3, 0x1.7e5c66dd54b33p+2, -0x1.0a1caa63db74dp-1, -0x1.47d3ba3d293c5p+3, 0x1.cc17c69d77dfbp+1, -0x1.0a1caa3c73073p-1, -0x1.45bd43843b5f6p+3, 0x1.c1b26525f5a85p+2, -0x1.0a1c8cb8c331fp-1, -0x1.4549c66fada5ap+3, 0x1.c6ad2baee2b9ep+2, 0x1.d81117c8006b0p-4, -0x1.3eb01d86355d7p+3, 0x1.9013bba7cd95ap+1, 0x1.d80f47273c242p-4, -0x1.3d6fcb1a4fa80p+3, 0x1.413dff542d754p+2, -0x1.0a1cac48441b6p-1, -0x1.3b85de6da8b76p+3, 0x1.6a0893e9f0fc7p+2, 0x1.d80feac5eb9bep-4, -0x1.396aef6105700p+3, 0x1.e49eeb50a916dp+1, 0x1.d80eac0d90db3p-4, -0x1.3749a8a7618cdp+3, 0x1.d58f6c3d895d9p+2, 0x1.d8105d32c6f7bp-4, -0x1.37391aa359aefp+3, 0x1.68c24bc37d629p+2, -0x1.0a1ca2a2c1e55p-1, -0x1.36e79304442a1p+3, 0x1.abb6db38ed71cp+1, -0x1.0a1c9e995eb9fp-1, -0x1.348d54ffed9cfp+3, 0x1.2f872e2cc2556p+2, 0x1.d8104b4372363p-4, -0x1.2fa0b0322c147p+3, 0x1.ab47f4c8e3ad4p+2, 0x1.d810dc6fbf12fp-4, -0x1.2e1c3fbef5e91p+3, 0x1.0098e906fb41fp+2, -0x1.0a1cc09b9c554p-1, -0x1.2dc1ccb615481p+3, 0x1.95b48a1a00a5dp+1, 0x1.d81223e8fedd1p-4, -0x1.2b2c2bd04b692p+3, 0x1.cb6ee37f4caa2p+2, -0x1.0a1cc1f1bf705p-1, -0x1.2b17da206b99bp+3, 0x1.8ca6393e46c34p+2, -0x1.0a1cc2aedb75ap-1, -0x1.2718631d95110p+3, 0x1.efcf7f25a9b5ep+1, 0x1.d8108c02a7573p-4, -0x1.25815cab3f48dp+3, 0x1.8b03604f694acp+1, -0x1.0a1cd0b91a15bp-1, -0x1.23723495d0062p+3, 0x1.5100170e39b30p+2, -0x1.0a1cc7ab9bea4p-1, -0x1.232c5be7d785ep+3, 0x1.e19f30099cd4dp+2, 0x1.d80f37e12f207p-4, -0x1.1ab19de7836a2p+3, 0x1.75ba930929b84p+1, 0x1.d80f366d3c3b6p-4, -0x1.1aa96bc66d46fp+3, 0x1.4d4ae318d3194p+2, 0x1.d810f7a82fb62p-4, -0x1.173e84024e76ep+3, 0x1.b0028563b7333p+2, -0x1.0a1cc7f15d78cp-1, -0x1.16f4178353651p+3, 0x1.21d8bc251b350p+2, 0x1.d80fb2b0bf04ap-4, -0x1.13d6272bddb42p+3, 0x1.1c29893e470d4p+2, -0x1.0a1ca49bfea17p-1, -0x1.12a9ce1d3901bp+3, 0x1.7c638d99a2b5cp+2, 0x1.d80fd2ae954cfp-4, -0x1.0e8cbfcf2da94p+3, 0x1.db672ea456dd4p+1, -0x1.0a1cb15955eaap-1, -0x1.0c39e93ee8f8bp+3, 0x1.c85d527860d3ep+1, 0x1.d81068dc18ab3p-4, -0x1.0b960f93d742cp+3, 0x1.b57e045862ceep+2, 0x1.d8102056e0038p-4, -0x1.0a1790ec56ac6p+3, 0x1.8ae80ae8a98f9p+2, -0x1.0a1cba8070cbap-1, -0x1.082ea8495ffa3p+3, 0x1.61a611e5bfd15p+1, -0x1.0a1cab930b6d5p-1, -0x1.05fcfc86c277ep+3, 0x1.4a6a8e65d604ap+2, -0x1.0a1cad7a7db93p-1, -0x1.fc82f8d985774p+2, 0x1.14a7f1a13a872p+2, 0x1.d8106564e1f22p-4, -0x1.f3829765c4298p+2, 0x1.4c8343b56a8b6p+2, 0x1.d810a5f4afad3p-4, -0x1.f10f59157af10p+2, 0x1.c34c2bf6db2dfp+1, 0x1.d80fe910ce56cp-4, -0x1.ef73b49da0115p+2, 0x1.b7f5fc5423911p+2, -0x1.0a1cb47f3e9dcp-1, -0x1.eb29348726520p+2, 0x1.74d2abe5ee08bp+2, -0x1.0a1cb1d8f0e3ep-1, -0x1.e7fe43fe46d4cp+2, 0x1.90a7bcf368352p+2, 0x1.d810619ac0b45p-4, -0x1.e454e24957da7p+2, 0x1.1804aa4c0b965p+2, -0x1.0a1cb8cbe27d1p-1, -0x1.d789cbf2dd9e7p+2, 0x1.c055234a6ce83p+2, 0x1.d81050a401baap-4, -0x1.d4d73bdfa454dp+2, 0x1.b6a6318799c14p+1, -0x1.0a1ca43502ab7p-1, -0x1.cc3ad12afb3e9p+2, 0x1.7996acc9153f7p+1, 0x1.d81096bf11673p-4, -0x1.c9a4c5188496dp+2, 0x1.538a95f31efe4p+2, -0x1.0a1caffdf4aafp-1, -0x1.c551abba94a90p+2, 0x1.66a8864c7c637p+2, 0x1.d810b6d0e2601p-4, -0x1.c32fb3003042bp+2, 0x1.16baadf55d20fp+2, 0x1.d8102ea5c120fp-4, -0x1.c040abcfc9be5p+2, 0x1.94ada0c865e51p+2, -0x1.0a1ca59c816f4p-1, -0x1.bac034e80055ap+2, 0x1.c85182e7b6c60p+2, -0x1.0a1cd33008e78p-1, -0x1.adc4dc6ebd3b7p+2, 0x1.02f14d0ba4736p+2, -0x1.0a1cb5f021e87p-1, -0x1.a19c509b323ccp+2, 0x1.b4ae218d892fap+2, 0x1.d810f48b4ed48p-4, -0x1.99fe70bb516dep+2, 0x1.8492f702eb91bp+1, -0x1.0a1cb1976a5a6p-1, -0x1.99d2eae615ec0p+2, 0x1.3d3a36ed4f51dp+2, -0x1.0a1cb69c6f3b6p-1, -0x1.95d794a6e8638p+2, 0x1.cbfa0a85787a4p+1, 0x1.d810722ffb6c4p-4, -0x1.903563cd98ed4p+2, 0x1.7b9eae5366c44p+2, -0x1.0a1cd6a15bf89p-1};
inline const int geneva_tag0_F[] = {3, 1, 15, 21, 9, 7, 14, 2, 5, 3, 19, 5, 4, 13, 7, 19, 3, 15, 27, 21, 13, 19, 14, 5, 14, 20, 6, 24, 8, 10, 14, 30, 34, 34, 20, 14, 30, 32, 42, 30, 19, 32, 27, 33, 21, 42, 34, 30, 54, 61, 51, 47, 51, 59, 38, 31, 44, 40, 33, 27, 10, 23, 24, 36, 25, 29, 32, 25, 36, 26, 21, 35, 22, 29, 25, 47, 59, 56, 56, 58, 49, 51, 47, 40, 42, 46, 34, 49, 35, 47, 41, 32, 36, 43, 41, 36, 22, 15, 12, 46, 37, 34, 61, 59, 51, 25, 32, 19, 24, 31, 38, 54, 45, 44, 57, 48, 46, 46, 48, 37, 19, 15, 25, 10, 8, 0, 54, 44, 55, 31, 24, 23, 1, 12, 15, 22, 25, 15, 53, 41, 43, 38, 27, 24, 34, 37, 20, 4, 8, 13, 26, 18, 21, 18, 9, 21, 41, 53, 42, 47, 56, 49, 30, 14, 19, 6, 20, 11, 20, 28, 16, 14, 6, 2, 42, 32, 41, 26, 35, 39, 39, 35, 49, 27, 38, 40, 54, 51, 45, 42, 53, 52, 33, 35, 21, 11, 20, 16, 17, 24, 27, 13, 17, 27, 45, 38, 44, 20, 37, 28, 45, 51, 40, 52, 46, 42, 45, 40, 38, 40, 47, 33, 21, 7, 13, 33, 47, 35, 8, 24, 17, 13, 8, 17, 53, 50, 60, 43, 50, 53, 57, 46, 52};
inline const GCell geneva_tag0_cells[] = {
    {{-0x1.2bcbbc52a291ap+3, 0x1.781a7728c3ae7p+2, 0x1.23e2b45360874p-5, -0x1.fe839cd2a83c8p+2, 0x1.7212ee76ba342p+2, -0x1.5e6bcd05693f9p-3, -0x1.15ece5b140707p+3, 0x1.21b207a5033cap+2, -0x1.89cd6eb86e93dp-3, -0x1.15ff771523811p+3, 0x1.65f9f2cd26464p+2, 0x1.0133d1012224bp+0}, 0x1.c8c5062d4f1a5p-2, 0x1.db46ead68af0ap-5},
};
struct GCase { const char* name; const double* V; int nV; const int* F; int nF; const GCell* cells; int ncells; };
inline const GCase geneva_cases[] = {
    {"geneva_run", geneva_run_V, 120, geneva_run_F, 129, geneva_run_cells, 10},
    {"geneva_tag0", geneva_tag0_V, 62, geneva_tag0_F, 81, geneva_tag0_cells, 1},
};

} // namespace exact_band_geneva_data
// clang-format on

/**
 * The Geneva base (inward, tag_0, exact rule): the 10 cells whose partition failed the tiling
 * check in the runs of ~/exact_folds/geneva (3 of 3 aborted in turn 1) and in reruns with the
 * partial fixes, on the run's own triangles, and a straddling cell on the tag_0 boundary of
 * m3.msh that failed it in the Python. Causes (clip()), each an open surface: a cap ring sorted by
 * angle skipped a vertex lying tol / theta inside the cap (the cut crossing a face at the tiny
 * angle theta); two faces' copies of one vertex, 1 tol apart, whose crossings on a nearly
 * parallel cut fell 6e-9 apart; a cap added where the kept faces already closed up. Each cell
 * must pass the checks (no throw) and match the Python.
 */
TEST_CASE("exact-band-integral-geneva", "[offset][exact_band]")
{
    using namespace exact_band_geneva_data;
    std::printf(
        "exact-band-integral-geneva: cells that failed the tiling check before the indexed-vertex "
        "clip\n");
    for (const GCase& c : geneva_cases) {
        Eigen::MatrixXd V(c.nV, 3);
        for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
        Eigen::MatrixXi F(c.nF, 3);
        for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
        const ExactBandIntegral eb(V, F);
        double worst = 0.;
        ExactBandIntegral::Stats st;
        st.check_closed = true;
        for (int k = 0; k < c.ncells; ++k) {
            const Tet T = tet_of(c.cells[k].T);
            INFO(c.name << " cell " << k);
            double I = 0.;
            REQUIRE_NOTHROW(I = eb.value(T, 0., &st));
            CHECK(rel(I, c.cells[k].I) <= 1e-12);
            worst = std::max(worst, rel(I, c.cells[k].I));
            REQUIRE_NOTHROW(eb.value(T, c.cells[k].delta));
        }
        CHECK(st.open_results == 0);
        std::printf(
            "  %-12s %d cells, %d triangles: max relative error vs the Python %.1e; %lld clip"
            " results checked closed, %lld open; %lld cuts closed up without a cap, %lld caps of"
            " several circuits\n",
            c.name,
            c.ncells,
            c.nF,
            worst,
            (long long)st.closed_checked,
            (long long)st.open_results,
            (long long)st.clips_closed_up,
            (long long)st.caps_split);
    }
}

/**
 * The offset side (ExactBandIntegral's class doc) against the Python (exact_mesh.band_integral,
 * value_grad_hess; data in exact_band_side_data.hpp): cells straddling the input surface of eleven
 * inputs, outward and inward, the triangles oriented out of the selection region, and the two
 * plates of side_counterexample.py. Checks: value and derivatives match the Python; a cell with
 * nothing clipped gives the unclipped value bit for bit; every clip result is closed; on the
 * plates a cell wholly inside the lower plate gives 0; the gradient and Hessian match central
 * differences on the clipped cells.
 *
 * Except on cells with a part where the members' planes disagree (D, ExactBandIntegral's class
 * doc): there the Python took the former D bound, an upper bound of the integral, where value()
 * now takes the side of the member nearest at the part's centroid (2026-10-14). On those the
 * check is that value() is at most the Python's bound and that value_grad_hess() gives the same
 * J. MEASURED against a quadrature of chi (d - delta) over the same pieces (scratchpad replay/,
 * 2026-10-14): the 24 plates cells that changed match it to 2.2e-18 delta vol where the Python
 * was up to 0.78 delta vol above it; the 4 geneva cells that changed moved by at most 2.6e-5
 * delta vol, below the quadrature's own accuracy there (1.2e-4).
 */
TEST_CASE("exact-band-integral-side", "[offset][exact_band]")
{
    using namespace exact_band_side_data;
    std::printf("exact-band-integral-side: the offset-side band integral against the Python\n");
    for (const SCase& c : side_cases) {
        Eigen::MatrixXd V(c.nV, 3);
        for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
        Eigen::MatrixXi F(c.nF, 3);
        for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
        const ExactBandIntegral eb(V, F, std::vector<int8_t>(size_t(c.nF), 1), c.sigma);
        const ExactBandIntegral plain(V, F);
        ExactBandIntegral::Stats st;
        st.check_closed = true;
        double e_J = 0., e_dJ = 0., e_G = 0., e_H = 0., fd_g = 0., fd_h = 0.;
        int n_clipped = 0, n_bits = 0, n_unclipped = 0, n_fd = 0, n_D_cells = 0;
        for (int k = 0; k < c.ncells; ++k) {
            const SCell& sc = c.cells[k];
            const Tet T = tet_of(sc.T);
            INFO(c.name << " cell " << k);
            const double vol = std::abs((T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]))) / 6.;
            const double unit = sc.delta * vol; // J is of size delta vol
            const int64_t D_before = st.side_D;
            const double J = eb.value(T, sc.delta, &st);
            // a part where the members' planes disagree: the Python's J is the former D bound
            const bool has_D = st.side_D > D_before;
            n_D_cells += has_D;
            if (has_D) {
                CHECK(J <= sc.J + 1e-12 * unit);
            } else {
                CHECK(std::abs(J - sc.J) <= 1e-12 * unit);
                e_J = std::max(e_J, std::abs(J - sc.J) / unit);
            }
            if (sc.clipped) {
                ++n_clipped;
            } else {
                ++n_unclipped;
                const bool same = J == plain.value(T, sc.delta);
                CHECK(same);
                n_bits += same;
            }
            Vec3 G;
            Eigen::Matrix3d H;
            const double Jd = eb.value_grad_hess(T, sc.kv, sc.delta, G, H);
            const Eigen::Map<const Eigen::Vector3d> Gr(sc.G);
            const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Hr(sc.H);
            const double eg = (G - Gr).norm() / std::max(Gr.norm(), 1e-300);
            const double eh =
                (H - Eigen::Matrix3d(Hr)).norm() / std::max(Eigen::Matrix3d(Hr).norm(), 1e-300);
            if (has_D) {
                CHECK(std::abs(Jd - J) <= 1e-12 * unit);
            } else {
                CHECK(std::abs(Jd - sc.Jd) <= 1e-12 * unit);
                CHECK(eg <= 1e-10);
                CHECK(eh <= 1e-9);
                e_dJ = std::max(e_dJ, std::abs(Jd - sc.Jd) / unit);
                e_G = std::max(e_G, eg);
                e_H = std::max(e_H, eh);
            }
            if (std::string(c.name) == "plates") {
                // a cell wholly inside the lower plate [0, 1]^2 x [0, 0.1] (true value 0)
                bool inside = true;
                for (const Vec3& p : T) {
                    inside = inside && p[0] >= 0. && p[0] <= 1. && p[1] >= 0. && p[1] <= 1. &&
                             p[2] >= 0. && p[2] <= 0.1;
                }
                if (inside) CHECK(J == 0.);
            }
            if (!sc.clipped || n_fd >= 24) continue;
            // central differences on clipped cells
            ++n_fd;
            double L = 0.;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < i; ++j) L = std::max(L, (T[size_t(i)] - T[size_t(j)]).norm());
            }
            double best_g = std::numeric_limits<double>::infinity(), best_h = best_g;
            for (const double h0 : {1e-4, 1e-5, 1e-6}) {
                const double h = h0 * L;
                Vec3 gf;
                Eigen::Matrix3d Hf;
                for (int d = 0; d < 3; ++d) {
                    Tet Tp = T, Tm = T;
                    Tp[size_t(sc.kv)][d] += h;
                    Tm[size_t(sc.kv)][d] -= h;
                    gf[d] = (eb.value(Tp, sc.delta) - eb.value(Tm, sc.delta)) / (2. * h);
                    Vec3 Gp, Gm;
                    Eigen::Matrix3d Hp, Hm;
                    eb.value_grad_hess(Tp, sc.kv, sc.delta, Gp, Hp);
                    eb.value_grad_hess(Tm, sc.kv, sc.delta, Gm, Hm);
                    Hf.col(d) = (Gp - Gm) / (2. * h);
                }
                best_g = std::min(best_g, (gf - G).norm() / std::max(G.norm(), 1e-300));
                best_h = std::min(best_h, (Hf - H).norm() / std::max(H.norm(), 1e-300));
            }
            CHECK(best_g <= 1e-6);
            CHECK(best_h <= 1e-4);
            fd_g = std::max(fd_g, best_g);
            fd_h = std::max(fd_h, best_h);
        }
        CHECK(st.open_results == 0);
        std::printf(
            "  %-13s %2d cells (%d clipped; D parts %lld, %lld kept, in %d cells): J vs the Python "
            "%.1e (delta vol);"
            " unclipped bit for bit %d/%d; derivatives J %.1e G %.1e H %.1e; central differences "
            "on %d"
            " clipped cells G %.1e H %.1e; %lld clip results closed, %lld open\n",
            c.name,
            c.ncells,
            n_clipped,
            (long long)st.side_D,
            (long long)st.side_D_pos,
            n_D_cells,
            e_J,
            n_bits,
            n_unclipped,
            e_dJ,
            e_G,
            e_H,
            n_fd,
            fd_g,
            fd_h,
            (long long)st.closed_checked,
            (long long)st.open_results);
    }
}

/**
 * Two configurations where value() was not the integral, found on 100033 (plates 2.9 thick, delta
 * 2.49; 2026-10-14), each against its exact value: next to a slab's face, inside its footprint and
 * away from its walls, that face is nearest everywhere, so J = vol (its mean distance - delta).
 *  - parallel faces: a box 10 x 2.9 x 10 and cells under it spanning 4.0, 2.5 and 1.5 along y, the
 *    faces given top first and bottom first. Below the box the two horizontal faces have one slope;
 *    faces_min_pieces() took them for one function and gave the piece to the lower index, so with
 *    the top face first and a span over 2.9 (the top face not pruned) value() was 0.
 *  - a grooved bottom: the box's bottom folded up 0.2 along x = 5, a reflex edge whose region
 *    opens up through the box; a cell above the top face spanning 4.0 has the groove edge for a
 *    member with the top face, its plane putting the cell on the far side: the part where the two
 *    disagree took the D bound, positive here against a true value below 0.
 */
TEST_CASE("exact-band-integral-plates", "[offset][exact_band]")
{
    const double delta = 2.4857853714681912, H = 2.9;
    // a solid of the triangles `tris`, its face sides from its centroid (both solids are
    // star-shaped about it)
    const auto build = [](const std::vector<Vec3>& P, const std::vector<std::array<int, 3>>& tris) {
        Eigen::MatrixXd V(int(P.size()), 3);
        Vec3 c = Vec3::Zero();
        for (size_t i = 0; i < P.size(); ++i) {
            V.row(int(i)) = P[i].transpose();
            c += P[i] / double(P.size());
        }
        Eigen::MatrixXi F(int(tris.size()), 3);
        std::vector<int8_t> side(tris.size());
        for (size_t f = 0; f < tris.size(); ++f) {
            const auto& t = tris[f];
            F.row(int(f)) << t[0], t[1], t[2];
            const Vec3 n =
                (P[size_t(t[1])] - P[size_t(t[0])]).cross(P[size_t(t[2])] - P[size_t(t[0])]);
            const Vec3 m = (P[size_t(t[0])] + P[size_t(t[1])] + P[size_t(t[2])]) / 3.;
            side[f] = n.dot(m - c) > 0 ? 1 : -1;
        }
        return ExactBandIntegral(V, F, side, 1);
    };
    const auto quad = [](std::vector<std::array<int, 3>>& tris, const std::array<int, 4>& q) {
        tris.push_back({q[0], q[1], q[2]});
        tris.push_back({q[0], q[2], q[3]});
    };
    // value() and value_grad_hess() against J = vol (mean of y_sign (y - y_face) - delta)
    const auto check = [&](const ExactBandIntegral& eb,
                           const Tet& T,
                           const double y_face,
                           const double y_sign,
                           const char* what,
                           ExactBandIntegral::Stats* st) {
        const double vol = std::abs((T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]))) / 6.;
        const double ybar = (T[0][1] + T[1][1] + T[2][1] + T[3][1]) / 4.;
        const double exact = vol * (y_sign * (ybar - y_face) - delta);
        const double unit = delta * vol;
        INFO(what);
        const double J = eb.value(T, delta, st);
        CHECK(std::abs(J - exact) <= 1e-12 * unit);
        Vec3 G;
        Eigen::Matrix3d Hs;
        const double Jd = eb.value_grad_hess(T, 3, delta, G, Hs);
        CHECK(std::abs(Jd - exact) <= 1e-12 * unit);
        // the gradient in the apex against central differences of value()
        Vec3 gf;
        const double h = 1e-5;
        for (int d = 0; d < 3; ++d) {
            Tet Tp = T, Tm = T;
            Tp[3][d] += h;
            Tm[3][d] -= h;
            gf[d] = (eb.value(Tp, delta) - eb.value(Tm, delta)) / (2. * h);
        }
        CHECK((G - gf).norm() <= 1e-6 * std::max(gf.norm(), 1.));
        std::printf(
            "  %-34s J %+.12f exact %+.12f (|diff| %.1e delta vol); gradient vs central "
            "differences %.1e\n",
            what,
            J,
            exact,
            std::abs(J - exact) / unit,
            (G - gf).norm() / std::max(gf.norm(), 1.));
    };
    std::printf("exact-band-integral-plates: value() on plates against the exact value\n");

    // parallel faces: the box [0, 10] x [0, H] x [0, 10], corner i = (x, y, z) bits of i
    std::vector<Vec3> P(8);
    for (int i = 0; i < 8; ++i) {
        P[size_t(i)] = Vec3(10. * (i & 1), H * ((i >> 1) & 1), 10. * ((i >> 2) & 1));
    }
    const std::array<int, 4> top = {2, 6, 7, 3}, bottom = {0, 1, 5, 4};
    const std::array<std::array<int, 4>, 4> walls = {
        {{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 2, 3, 1}, {4, 5, 7, 6}}};
    for (const bool top_first : {true, false}) {
        std::vector<std::array<int, 3>> tris;
        quad(tris, top_first ? top : bottom);
        quad(tris, top_first ? bottom : top);
        for (const auto& q : walls) quad(tris, q);
        const ExactBandIntegral eb = build(P, tris);
        for (const double y_lo : {-4.5, -3., -2.}) {
            const Tet T = {
                {Vec3(3., -0.5, 3.), Vec3(7., -0.5, 3.5), Vec3(4., -0.5, 7.), Vec3(5., y_lo, 5.)}};
            const std::string what =
                std::string(top_first ? "top face first" : "bottom face first") + ", span " +
                std::to_string(-0.5 - y_lo).substr(0, 3);
            check(eb, T, 0., -1., what.c_str(), nullptr);
        }
    }

    // a grooved bottom: y = 0 at x = 0 and 10, y = g along x = 5 (the edge 1-4)
    const double g = 0.2;
    const std::vector<Vec3> Q = {
        Vec3(0., 0., 0.),
        Vec3(5., g, 0.),
        Vec3(10., 0., 0.),
        Vec3(0., 0., 10.),
        Vec3(5., g, 10.),
        Vec3(10., 0., 10.),
        Vec3(0., H, 0.),
        Vec3(10., H, 0.),
        Vec3(0., H, 10.),
        Vec3(10., H, 10.)};
    std::vector<std::array<int, 3>> tris;
    quad(tris, {0, 1, 4, 3}); // the bottom's two halves, meeting at the groove
    quad(tris, {1, 2, 5, 4});
    quad(tris, {6, 8, 9, 7}); // top
    quad(tris, {0, 3, 8, 6}); // x = 0
    quad(tris, {2, 7, 9, 5}); // x = 10
    // the z = 0 and z = 10 walls, fans from the groove's ends
    for (const std::array<int, 3>& t : {std::array<int, 3>{1, 2, 7}, {1, 7, 6}, {1, 6, 0}}) {
        tris.push_back(t);
    }
    for (const std::array<int, 3>& t : {std::array<int, 3>{4, 5, 9}, {4, 9, 8}, {4, 8, 3}}) {
        tris.push_back(t);
    }
    const ExactBandIntegral eb = build(Q, tris);
    const Tet T = {
        {Vec3(4., H + 0.5, 4.),
         Vec3(6., H + 0.5, 4.5),
         Vec3(5., H + 0.5, 6.5),
         Vec3(5., H + 4.5, 5.)}};
    ExactBandIntegral::Stats st;
    check(eb, T, H, 1., "grooved bottom, cell above, span 4.0", &st);
    // the case exercises a part where the members' planes disagree
    CHECK(st.side_D > 0);
    std::printf("  grooved bottom: %lld parts where the planes disagree\n", (long long)st.side_D);
}

namespace {

/// Point-triangle distance, written apart from the code under test (test_exact_tri.dist_ref):
/// min over the three segments, and the plane distance when the projection is inside.
double ref_point_tri(const Vec3& x, const Vec3& a, const Vec3& b, const Vec3& c)
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [p, q] : {std::pair{&a, &b}, std::pair{&b, &c}, std::pair{&c, &a}}) {
        const Vec3 d = *q - *p;
        const double s = std::clamp((x - *p).dot(d) / d.dot(d), 0., 1.);
        best = std::min(best, (x - *p - s * d).norm());
    }
    const Vec3 v0 = b - a, v1 = c - a, w = x - a;
    Eigen::Matrix2d G;
    G << v0.dot(v0), v0.dot(v1), v0.dot(v1), v1.dot(v1);
    const Eigen::Vector2d lam = G.ldlt().solve(Eigen::Vector2d(w.dot(v0), w.dot(v1)));
    if (lam[0] >= 0 && lam[1] >= 0 && lam[0] + lam[1] <= 1) {
        const Vec3 n = v0.cross(v1).normalized();
        best = std::min(best, std::abs(w.dot(n)));
    }
    return best;
}

double ref_distance(const Vec3& x, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    double d = std::numeric_limits<double>::infinity();
    for (int f = 0; f < F.rows(); ++f) {
        d = std::min(d, ref_point_tri(x, V.row(F(f, 0)), V.row(F(f, 1)), V.row(F(f, 2))));
    }
    return d;
}

} // namespace

/**
 * ExactBandIntegral::max_distance against the Python (exact_mesh.max_on_triangle, data in
 * exact_band_max_data.hpp: faces of test_max_on_triangle.py near seven inputs, and the 8 cube
 * faces of max cube_out turn 28) and against dense sampling on the embedded input subset (every
 * face's candidates and the faces their regions are built from: every point of f has its nearest
 * triangle there). Units: eps (|x| + delta), the rounding unit of a distance at coordinates |x|.
 *  - parity: the max, U and the partition's counts equal the Python's;
 *  - upper bound: d on a 24-per-side barycentric grid never exceeds the max;
 *  - exact: d at the returned point equals the max, where it does not come from a rule-(a) piece;
 *  - cube8: the former settle bound (U = min_max_corner_distance()) marked all 8
 *    faces, the exact max marks none.
 */
TEST_CASE("exact-band-max-distance", "[offset][exact_band]")
{
    using namespace exact_band_max_data;
    constexpr double EPS = std::numeric_limits<double>::epsilon();
    std::printf(
        "exact-band-max-distance: max_distance against the Python and a 24-per-side grid\n");
    for (const MaxCase& c : max_cases) {
        Eigen::MatrixXd V(c.nV, 3);
        for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
        Eigen::MatrixXi F(c.nF, 3);
        for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
        const ExactBandIntegral eb(V, F);
        const auto tris = triangle_envelope(V, F);
        const double pad = input_pad(V);
        double w_par = 0., w_U = 0., w_at = 0., w_ub = -std::numeric_limits<double>::infinity(),
               w_eq = 0.;
        int n_ra = 0;
        const bool cube8 = std::string(c.name) == "cube8";
        if (cube8)
            std::printf("  cube8, bars: face  former (U - delta)/eps  exact (max - delta)/eps\n");
        for (int k = 0; k < c.nfaces; ++k) {
            const MaxFace& mf = c.faces[k];
            const std::array<Vec3, 3> P = {
                {Vec3(mf.P[0], mf.P[1], mf.P[2]),
                 Vec3(mf.P[3], mf.P[4], mf.P[5]),
                 Vec3(mf.P[6], mf.P[7], mf.P[8])}};
            ExactBandIntegral::MaxInfo info;
            const double m = eb.max_distance(P, &info);
            double xs = 0.;
            for (const Vec3& p : P) xs = std::max(xs, p.cwiseAbs().maxCoeff());
            const double unit = EPS * (xs + c.delta);
            INFO(c.name << " face " << k << " (" << mf.label << ")");
            const double par = std::abs(m - mf.dmax) / unit;
            CHECK(par <= 64.);
            CHECK(std::abs(info.U - mf.U) <= 64. * unit);
            CHECK(info.rule_a == bool(mf.rule_a));
            CHECK(info.pieces == mf.pieces);
            CHECK(info.candidates == mf.candidates);
            CHECK(info.rule_a_pieces == mf.rule_a_pieces);
            w_par = std::max(w_par, par);
            w_U = std::max(w_U, std::abs(info.U - mf.U) / unit);
            w_at = std::max(
                w_at,
                (info.at - Vec3(mf.at[0], mf.at[1], mf.at[2])).cwiseAbs().maxCoeff() / (EPS * xs));
            constexpr int N = 24;
            double smax = 0.;
            for (int i = 0; i <= N; ++i) {
                for (int j = 0; i + j <= N; ++j) {
                    const Vec3 x =
                        P[0] + (double(i) / N) * (P[1] - P[0]) + (double(j) / N) * (P[2] - P[0]);
                    smax = std::max(smax, ref_distance(x, V, F));
                }
            }
            CHECK(smax - m <= 64. * unit);
            w_ub = std::max(w_ub, (smax - m) / unit);
            if (info.rule_a) {
                ++n_ra;
                CHECK(ref_distance(info.at, V, F) - m <= 64. * unit);
            } else {
                const double e = std::abs(ref_distance(info.at, V, F) - m) / unit;
                CHECK(e <= 64.);
                w_eq = std::max(w_eq, e);
            }
            if (cube8) {
                const double former = min_max_corner_distance(*tris, pad, P.data(), 3);
                CHECK(
                    std::abs(former - info.U) <= 64. * unit); // the settle loop's former bound is U
                const double ob = (former - c.delta) / c.eps, nb = (m - c.delta) / c.eps;
                std::printf("    %-22s %8.4f %10.4f\n", mf.label, ob, nb);
                CHECK(ob > 1.);
                CHECK(nb <= 1.);
            }
        }
        std::printf(
            "  %-13s %2d faces (%d rule-(a) maxima): parity max %.2f, U %.2f, at %.2f (eps units); "
            "grid -"
            " max %+.2f; |d(at) - max| %.2f\n",
            c.name,
            c.nfaces,
            n_ra,
            w_par,
            w_U,
            w_at,
            w_ub,
            w_eq);
    }
}

/**
 * ExactBandIntegral::max_distance on front faces nearly parallel to a side plane of an input
 * triangle's region (exact_band_parallel_data.hpp: the faces 70565's debug1 and debug2 aborted
 * on, 0.08 and 0.35 degrees off it, with the input around them). The cut by that plane snapped
 * two neighbouring ring vertices onto it and both parts kept the edge between them, so the pieces
 * counted a sliver twice; it is tol / sin(theta) wide across the face, and the former tiling
 * bound (2 sqrt(3) tol D) ignored the angle: max_distance() threw. Units as
 * exact-band-max-distance.
 *  - no throw;
 *  - upper bound: d on a 24-per-side barycentric grid never exceeds the max;
 *  - exact: d at the returned point equals the max, where it does not come from a rule-(a) piece.
 */
TEST_CASE("exact-band-max-distance-parallel", "[offset][exact_band]")
{
    using namespace exact_band_parallel_data;
    constexpr double EPS = std::numeric_limits<double>::epsilon();
    const double delta = 0x1.08e27cb760a9fp-8; // 70565's target_distance, 0.004041820000000001
    std::printf(
        "exact-band-max-distance-parallel: faces nearly parallel to a side plane of a region\n");
    for (const ParallelCase& c : parallel_cases) {
        Eigen::MatrixXd V(c.nV, 3);
        for (int i = 0; i < c.nV; ++i) V.row(i) << c.V[3 * i], c.V[3 * i + 1], c.V[3 * i + 2];
        Eigen::MatrixXi F(c.nF, 3);
        for (int i = 0; i < c.nF; ++i) F.row(i) << c.F[3 * i], c.F[3 * i + 1], c.F[3 * i + 2];
        const ExactBandIntegral eb(V, F);
        const std::array<Vec3, 3> P = {
            {Vec3(c.P[0], c.P[1], c.P[2]),
             Vec3(c.P[3], c.P[4], c.P[5]),
             Vec3(c.P[6], c.P[7], c.P[8])}};
        INFO(c.name);
        ExactBandIntegral::MaxInfo info;
        double m = 0.;
        REQUIRE_NOTHROW(m = eb.max_distance(P, &info));
        double xs = 0.;
        for (const Vec3& p : P) xs = std::max(xs, p.cwiseAbs().maxCoeff());
        const double unit = EPS * (xs + delta);
        constexpr int N = 24;
        double smax = 0.;
        for (int i = 0; i <= N; ++i) {
            for (int j = 0; i + j <= N; ++j) {
                const Vec3 x =
                    P[0] + (double(i) / N) * (P[1] - P[0]) + (double(j) / N) * (P[2] - P[0]);
                smax = std::max(smax, ref_distance(x, V, F));
            }
        }
        CHECK(smax - m <= 64. * unit);
        const double e = ref_distance(info.at, V, F) - m;
        if (info.rule_a) {
            CHECK(e <= 64. * unit);
        } else {
            CHECK(std::abs(e) <= 64. * unit);
        }
        std::printf(
            "  %-17s max %.6f delta, %d candidates, %d pieces; grid - max %+.2f, d(at) - max "
            "%+.2f (eps units)\n",
            c.name,
            m / delta,
            info.candidates,
            info.pieces,
            (smax - m) / unit,
            e / unit);
    }
}

/**
 * Hidden: EXACT_BAND_BENCH_DIR=<dir> ./wmtk_test_topological_offset "[.bench_max]". Reads
 * <dir>/<case>_max.bin (gen_max_data.py: the full input and every face of test_max_on_triangle.py
 * with the Python's max and U), prints the parity over all faces and the time per face of
 * max_distance() and of the former bound (min_max_corner_distance()) on them.
 */
TEST_CASE("exact-band-max-bench", "[.bench_max]")
{
    const char* dir = std::getenv("EXACT_BAND_BENCH_DIR");
    if (!dir) {
        WARN("EXACT_BAND_BENCH_DIR not set");
        return;
    }
    constexpr double EPS = std::numeric_limits<double>::epsilon();
    using clk = std::chrono::steady_clock;
    for (const char* name :
         {"cube_out",
          "cylinder_out",
          "prism_in",
          "slot",
          "t100026",
          "sphere",
          "cylinder_in",
          "cube8"}) {
        std::ifstream in(std::string(dir) + "/" + name + "_max.bin", std::ios::binary);
        REQUIRE(in.good());
        int32_t hdr[3];
        double de[2];
        in.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
        in.read(reinterpret_cast<char*>(de), sizeof(de));
        const int nV = hdr[0], nF = hdr[1], n = hdr[2];
        std::vector<double> Vb(static_cast<size_t>(3 * nV)), Pb(static_cast<size_t>(9 * n)),
            dm(static_cast<size_t>(n)), Ub(static_cast<size_t>(n));
        std::vector<int32_t> Fb(static_cast<size_t>(3 * nF)), ra(static_cast<size_t>(n));
        const auto rd = [&](auto& v) {
            in.read(reinterpret_cast<char*>(v.data()), std::streamsize(v.size() * sizeof(v[0])));
        };
        rd(Vb);
        rd(Fb);
        rd(Pb);
        rd(dm);
        rd(Ub);
        rd(ra);
        REQUIRE(in.good());
        Eigen::MatrixXd V(nV, 3);
        Eigen::MatrixXi F(nF, 3);
        for (int i = 0; i < nV; ++i)
            V.row(i) << Vb[size_t(3 * i)], Vb[size_t(3 * i + 1)], Vb[size_t(3 * i + 2)];
        for (int i = 0; i < nF; ++i)
            F.row(i) << Fb[size_t(3 * i)], Fb[size_t(3 * i + 1)], Fb[size_t(3 * i + 2)];
        const ExactBandIntegral eb(V, F);
        const auto tris = triangle_envelope(V, F);
        const double pad = input_pad(V);
        std::vector<std::array<Vec3, 3>> faces(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < 3; ++j) {
                faces[size_t(i)][size_t(j)] = Vec3(
                    Pb[size_t(9 * i + 3 * j)],
                    Pb[size_t(9 * i + 3 * j + 1)],
                    Pb[size_t(9 * i + 3 * j + 2)]);
            }
        }
        double w_par = 0., w_U = 0.;
        int n_ra_mismatch = 0;
        for (int i = 0; i < n; ++i) {
            ExactBandIntegral::MaxInfo info;
            const double m = eb.max_distance(faces[size_t(i)], &info);
            double xs = 0.;
            for (const Vec3& p : faces[size_t(i)]) xs = std::max(xs, p.cwiseAbs().maxCoeff());
            const double unit = EPS * (xs + de[0]);
            w_par = std::max(w_par, std::abs(m - dm[size_t(i)]) / unit);
            w_U = std::max(w_U, std::abs(info.U - Ub[size_t(i)]) / unit);
            n_ra_mismatch += info.rule_a != bool(ra[size_t(i)]);
        }
        CHECK(w_par <= 64.);
        CHECK(n_ra_mismatch == 0);
        constexpr int REP = 5;
        double sink = 0.;
        std::vector<double> us(static_cast<size_t>(n), 0.), ub(static_cast<size_t>(n), 0.);
        for (int r = 0; r < REP; ++r) {
            for (int i = 0; i < n; ++i) {
                auto a = clk::now();
                sink += eb.max_distance(faces[size_t(i)]);
                us[size_t(i)] += 1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
                a = clk::now();
                sink += min_max_corner_distance(*tris, pad, faces[size_t(i)].data(), 3);
                ub[size_t(i)] += 1e6 * std::chrono::duration<double>(clk::now() - a).count() / REP;
            }
        }
        const auto pct = [](std::vector<double> v, double p) {
            std::sort(v.begin(), v.end());
            return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
        };
        double mean = 0., mean_b = 0.;
        for (int i = 0; i < n; ++i) {
            mean += us[size_t(i)] / n;
            mean_b += ub[size_t(i)] / n;
        }
        std::printf(
            "== %-12s %4d faces, %4d triangles: parity max %.2f, U %.2f (eps units), rule_a "
            "mismatches %d"
            " | max_distance us per face: mean %.2f p50 %.2f p90 %.2f p99 %.2f max %.2f | former "
            "bound"
            " mean %.2f%s\n",
            name,
            n,
            nF,
            w_par,
            w_U,
            n_ra_mismatch,
            mean,
            pct(us, 0.5),
            pct(us, 0.9),
            pct(us, 0.99),
            pct(us, 1.),
            mean_b,
            sink == 0. ? " " : "");
    }
}

/**
 * Hidden replay: EXACT_BAND_REPLAY_TRIS=<file> EXACT_BAND_REPLAY_CELLS=<file>
 * ./wmtk_test_topological_offset "[.replay]". The triangles as ExactBandIntegral received them in a
 * run ("nV nF", nV lines of hex coordinates, nF index lines); one cell per line of CELLS (12 hex
 * corners, then delta). Prints each cell's value() and its partition counts, or the error.
 */
TEST_CASE("exact-band-replay", "[.replay]")
{
    const char* tf = std::getenv("EXACT_BAND_REPLAY_TRIS");
    const char* cf = std::getenv("EXACT_BAND_REPLAY_CELLS");
    if (!tf || !cf) {
        WARN("EXACT_BAND_REPLAY_TRIS / EXACT_BAND_REPLAY_CELLS not set");
        return;
    }
    std::ifstream in(tf);
    REQUIRE(in.good());
    int nV = 0, nF = 0;
    in >> nV >> nF;
    Eigen::MatrixXd V(nV, 3);
    Eigen::MatrixXi F(nF, 3);
    std::string a, b, c;
    for (int i = 0; i < nV; ++i) {
        in >> a >> b >> c;
        V.row(i) << std::strtod(a.c_str(), nullptr), std::strtod(b.c_str(), nullptr),
            std::strtod(c.c_str(), nullptr);
    }
    for (int i = 0; i < nF; ++i) in >> F(i, 0) >> F(i, 1) >> F(i, 2);
    REQUIRE(in.good());
    const ExactBandIntegral eb(V, F);
    std::ifstream cells(cf);
    std::string line;
    int k = 0;
    while (std::getline(cells, line)) {
        if (line.empty()) continue;
        std::istringstream ls(line);
        double x[13];
        for (double& v : x) {
            std::string tok;
            ls >> tok;
            v = std::strtod(tok.c_str(), nullptr);
        }
        Tet T;
        for (int i = 0; i < 4; ++i) T[size_t(i)] = Vec3(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
        ExactBandIntegral::Stats st;
        try {
            const double J = eb.value(T, x[12], &st);
            std::printf(
                "cell %d: J %.17g | candidates %lld pieces %lld clips %lld\n",
                k,
                J,
                (long long)st.candidates,
                (long long)st.pieces,
                (long long)st.clips);
        } catch (const std::exception& e) {
            std::printf(
                "cell %d: ERROR %s | candidates %lld clips %lld\n",
                k,
                e.what(),
                (long long)st.candidates,
                (long long)st.clips);
        }
        ++k;
    }
}

namespace {

/// A stored band (exact-band-hessian-states): the input triangles, delta, and the band cells as
/// vertex ids into P (vertices merged by exact coordinates).
struct BandState
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    double delta = 0.;
    std::vector<Vec3> P;
    std::vector<std::array<int, 4>> cell;
};

/// int32 nV, nF, nT; double delta; V (nV x 3 doubles); F (nF x 3 int32); nT cells of 12 doubles.
BandState read_band_state(const char* file)
{
    std::ifstream in(file, std::ios::binary);
    REQUIRE(in.good());
    int32_t hdr[3];
    BandState s;
    in.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
    in.read(reinterpret_cast<char*>(&s.delta), sizeof(s.delta));
    const int nV = hdr[0], nF = hdr[1], nT = hdr[2];
    std::vector<double> Vb(size_t(3 * nV)), Tb(size_t(12 * nT));
    std::vector<int32_t> Fb(size_t(3 * nF));
    in.read(reinterpret_cast<char*>(Vb.data()), std::streamsize(Vb.size() * sizeof(double)));
    in.read(reinterpret_cast<char*>(Fb.data()), std::streamsize(Fb.size() * sizeof(int32_t)));
    in.read(reinterpret_cast<char*>(Tb.data()), std::streamsize(Tb.size() * sizeof(double)));
    REQUIRE(in.good());
    s.V.resize(nV, 3);
    for (int i = 0; i < nV; ++i)
        s.V.row(i) << Vb[size_t(3 * i)], Vb[size_t(3 * i + 1)], Vb[size_t(3 * i + 2)];
    s.F.resize(nF, 3);
    for (int i = 0; i < nF; ++i)
        s.F.row(i) << Fb[size_t(3 * i)], Fb[size_t(3 * i + 1)], Fb[size_t(3 * i + 2)];
    std::vector<std::pair<std::array<double, 3>, int>> pts;
    for (int t = 0; t < nT; ++t) {
        for (int j = 0; j < 4; ++j) {
            const double* p = &Tb[size_t(12 * t + 3 * j)];
            pts.push_back({{p[0], p[1], p[2]}, 4 * t + j});
        }
    }
    std::sort(pts.begin(), pts.end());
    s.cell.resize(size_t(nT));
    for (size_t i = 0; i < pts.size(); ++i) {
        if (i == 0 || pts[i].first != pts[i - 1].first)
            s.P.push_back(Vec3(pts[i].first[0], pts[i].first[1], pts[i].first[2]));
        s.cell[size_t(pts[i].second / 4)][size_t(pts[i].second % 4)] = int(s.P.size()) - 1;
    }
    return s;
}

/// The one-vertex energy of the "exact" rule at x = the moving corner: sum over the ring cells
/// of value_grad_hess() (x in its slot kv of each cell).
struct RingCell
{
    Tet T;
    int kv = 0;
};
void ring_grad_hess(
    const ExactBandIntegral& eb,
    const std::vector<RingCell>& ring,
    const Vec3& x,
    const double delta,
    Vec3& G,
    Eigen::Matrix3d& H)
{
    G.setZero();
    H.setZero();
    for (const RingCell& rc : ring) {
        Tet T = rc.T;
        T[size_t(rc.kv)] = x;
        Vec3 g;
        Eigen::Matrix3d h;
        eb.value_grad_hess(T, rc.kv, delta, g, h);
        G += g;
        H += h;
    }
}

/// |H_fd - H| / |H| for central differences of the gradient at the steps hs (absolute).
std::vector<double> ring_fd_errors(
    const ExactBandIntegral& eb,
    const std::vector<RingCell>& ring,
    const Vec3& x,
    const double delta,
    const std::vector<double>& hs,
    double* kink = nullptr)
{
    Vec3 G;
    Eigen::Matrix3d H;
    ring_grad_hess(eb, ring, x, delta, G, H);
    std::vector<double> out;
    for (const double h : hs) {
        Eigen::Matrix3d Hf;
        for (int k = 0; k < 3; ++k) {
            Vec3 gp, gm;
            Eigen::Matrix3d hp, hm;
            ring_grad_hess(eb, ring, x + h * Vec3::Unit(k), delta, gp, hp);
            ring_grad_hess(eb, ring, x - h * Vec3::Unit(k), delta, gm, hm);
            Hf.col(k) = (gp - gm) / (2. * h);
        }
        out.push_back((Hf - H).norm() / std::max(H.norm(), 1e-300));
    }
    if (kink) { // |H_fwd - H_bwd| / |H| at the step hs[2]: a kink of the gradient at x
        const double h = hs[2];
        Eigen::Matrix3d Hp, Hm;
        for (int k = 0; k < 3; ++k) {
            Vec3 gp, gm;
            Eigen::Matrix3d hp, hm;
            ring_grad_hess(eb, ring, x + h * Vec3::Unit(k), delta, gp, hp);
            ring_grad_hess(eb, ring, x - h * Vec3::Unit(k), delta, gm, hm);
            Hp.col(k) = (gp - G) / h;
            Hm.col(k) = (G - gm) / h;
        }
        *kink = (Hp - Hm).norm() / std::max(H.norm(), 1e-300);
    }
    return out;
}

} // namespace

namespace {
/// A stored cube band of turn 1 (offset outward): the input, 14 vertices (8 corners, 6 face
/// centres) and 24 triangles, and the 15 cells of one front vertex x, which lies on the symmetry
/// plane x = y (exact-band-integral-hessian-contact).
// clang-format off
constexpr double kContactDelta = 0x1.1bc779ef0d4e8p-5;
constexpr double kContactV[] = {
    -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
    -0x1.0000000000000p-1, -0x1.0000000000000p-1, 0x1.0000000000000p-1,
    -0x1.0000000000000p-1, 0x0.0p+0, 0x0.0p+0,
    -0x1.0000000000000p-1, 0x1.0000000000000p-1, -0x1.0000000000000p-1,
    -0x1.0000000000000p-1, 0x1.0000000000000p-1, 0x1.0000000000000p-1,
    0x0.0p+0, -0x1.0000000000000p-1, 0x0.0p+0,
    0x0.0p+0, 0x0.0p+0, -0x1.0000000000000p-1,
    0x0.0p+0, 0x0.0p+0, 0x1.0000000000000p-1,
    0x0.0p+0, 0x1.0000000000000p-1, 0x0.0p+0,
    0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
    0x1.0000000000000p-1, -0x1.0000000000000p-1, 0x1.0000000000000p-1,
    0x1.0000000000000p-1, 0x0.0p+0, 0x0.0p+0,
    0x1.0000000000000p-1, 0x1.0000000000000p-1, -0x1.0000000000000p-1,
    0x1.0000000000000p-1, 0x1.0000000000000p-1, 0x1.0000000000000p-1,
};
constexpr int kContactF[] = {
    2, 0, 1, 0, 5, 1, 2, 1, 4, 7, 4, 1,
    5, 10, 1, 7, 1, 10, 0, 2, 3, 0, 3, 6,
    5, 0, 9, 6, 9, 0, 2, 4, 3, 3, 4, 8,
    13, 8, 4, 13, 4, 7, 3, 8, 12, 6, 3, 12,
    10, 9, 11, 10, 5, 9, 13, 10, 11, 13, 7, 10,
    12, 11, 9, 12, 9, 6, 11, 12, 13, 8, 13, 12,
};
/// Per cell its 4 corners, then the slot of x among them.
constexpr double kContactRing[][13] = {
    {
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.0a3d70a3d70a4p-1, -0x1.0a3d70a3d70a4p-1, -0x1.0a3d70a3d70a4p-1,
        -0x1.0c76c19b42d22p-1, -0x1.e7127cc97a5bcp-2, -0x1.0c76c19b42d22p-1,
        1},
    {
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.8000000000000p-2, -0x1.8000000000000p-2, -0x1.0000000000000p-1,
        -0x1.c89bde16d8882p-2, -0x1.f1c480592d32dp-2, -0x1.11a3b340ffb6ep-1,
        1},
    {
        -0x1.c89bde16d8882p-2, -0x1.f1c480592d32dp-2, -0x1.11a3b340ffb6ep-1,
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.e7127cc97a5bcp-2, -0x1.0c76c19b42d22p-1, -0x1.0c76c19b42d22p-1,
        2},
    {
        -0x1.ee5907c8bbb0fp-2, -0x1.cb0b175a3312ep-2, -0x1.11a6f837444f1p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.83327d842a3dep-2, -0x1.7a6248ef1bf90p-2, -0x1.11ae688e51928p-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        1},
    {
        -0x1.0000000000000p-1, -0x1.11bc779ef0d4ep-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.e7127cc97a5bcp-2, -0x1.0c76c19b42d22p-1, -0x1.0c76c19b42d22p-1,
        1},
    {
        -0x1.0a3d70a3d70a4p-1, -0x1.0a3d70a3d70a4p-1, -0x1.0a3d70a3d70a4p-1,
        -0x1.0000000000000p-1, -0x1.11bc779ef0d4ep-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        2},
    {
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.8737ae835726ep-2, -0x1.8737ae835726ep-2, -0x1.11b06f410de05p-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        -0x1.79bfaf56e1458p-2, -0x1.84110bd449126p-2, -0x1.11b25154f6ec7p-1,
        0},
    {
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.8737ae835726ep-2, -0x1.8737ae835726ep-2, -0x1.11b06f410de05p-1,
        -0x1.83327d842a3dep-2, -0x1.7a6248ef1bf90p-2, -0x1.11ae688e51928p-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        0},
    {
        -0x1.0c76c19b42d22p-1, -0x1.e7127cc97a5bcp-2, -0x1.0c76c19b42d22p-1,
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.ee5907c8bbb0fp-2, -0x1.cb0b175a3312ep-2, -0x1.11a6f837444f1p-1,
        2},
    {
        -0x1.0000000000000p-1, -0x1.0000000000000p-1, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.ee5907c8bbb0fp-2, -0x1.cb0b175a3312ep-2, -0x1.11a6f837444f1p-1,
        -0x1.8000000000000p-2, -0x1.8000000000000p-2, -0x1.0000000000000p-1,
        1},
    {
        -0x1.c89bde16d8882p-2, -0x1.f1c480592d32dp-2, -0x1.11a3b340ffb6ep-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        -0x1.79bfaf56e1458p-2, -0x1.84110bd449126p-2, -0x1.11b25154f6ec7p-1,
        1},
    {
        -0x1.7f31977bc2125p-2, -0x1.7f31977bc2125p-2, -0x1.08d9898286f56p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        -0x1.c89bde16d8882p-2, -0x1.f1c480592d32dp-2, -0x1.11a3b340ffb6ep-1,
        1},
    {
        -0x1.7f31977bc2125p-2, -0x1.7f31977bc2125p-2, -0x1.08d9898286f56p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.ee5907c8bbb0fp-2, -0x1.cb0b175a3312ep-2, -0x1.11a6f837444f1p-1,
        -0x1.7eca6339a31b8p-2, -0x1.7eca6339a31b8p-2, -0x1.0d464e43ca701p-1,
        1},
    {
        -0x1.8000000000000p-2, -0x1.8000000000000p-2, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.7f31977bc2125p-2, -0x1.7f31977bc2125p-2, -0x1.08d9898286f56p-1,
        -0x1.c89bde16d8882p-2, -0x1.f1c480592d32dp-2, -0x1.11a3b340ffb6ep-1,
        1},
    {
        -0x1.8000000000000p-2, -0x1.8000000000000p-2, -0x1.0000000000000p-1,
        -0x1.fe632ef78424ap-2, -0x1.fe632ef78424ap-2, -0x1.11b313050deacp-1,
        -0x1.ee5907c8bbb0fp-2, -0x1.cb0b175a3312ep-2, -0x1.11a6f837444f1p-1,
        -0x1.7f31977bc2125p-2, -0x1.7f31977bc2125p-2, -0x1.08d9898286f56p-1,
        1},
};
// clang-format on
} // namespace

/**
 * The one-vertex Hessian where a face of a cell through x holds a line on which two fixed planes
 * of the partition meet (ExactBandIntegral::value_grad_hess(), the cut terms of a piece touching
 * that face along an edge only): the ring of a front vertex x of the stored cube band on the
 * symmetry plane x = y. Two of its cells have the face (x, v, .) in that plane, v the cube's corner
 * (-1/2, -1/2, -1/2), which holds the line x = y = -1/2 below v, where the bottom face's prism,
 * the cone of v and an edge's region meet. Checks every cell's Hessian and the ring's against
 * central differences of the gradient (min over h = 1e-5, 1e-6 delta). Before the edge-contact
 * term: those two cells 98% and 94% off, the ring 57%; the other cells within 4e-7.
 */
TEST_CASE("exact-band-integral-hessian-contact", "[offset][exact_band]")
{
    const int nV = int(std::size(kContactV)) / 3, nF = int(std::size(kContactF)) / 3;
    Eigen::MatrixXd V(nV, 3);
    for (int i = 0; i < nV; ++i)
        V.row(i) << kContactV[3 * i], kContactV[3 * i + 1], kContactV[3 * i + 2];
    Eigen::MatrixXi F(nF, 3);
    for (int i = 0; i < nF; ++i)
        F.row(i) << kContactF[3 * i], kContactF[3 * i + 1], kContactF[3 * i + 2];
    // the solid's side from the triangles' orientation (out of the cube here), the band outside
    double vol6 = 0.;
    for (int i = 0; i < nF; ++i) {
        const Vec3 a = V.row(F(i, 0)), b = V.row(F(i, 1)), c = V.row(F(i, 2));
        vol6 += a.dot(b.cross(c));
    }
    const ExactBandIntegral eb(V, F, std::vector<int8_t>(size_t(nF), vol6 > 0. ? 1 : -1), 1);
    const double delta = kContactDelta;
    std::vector<RingCell> ring;
    for (const auto& r : kContactRing) {
        RingCell rc;
        rc.T = tet_of(r);
        rc.kv = int(r[12]);
        ring.push_back(rc);
    }
    const Vec3 x = ring[0].T[size_t(ring[0].kv)];
    const std::vector<double> hs = {1e-5 * delta, 1e-6 * delta};
    double worst = 0.;
    for (const RingCell& rc : ring) {
        const std::vector<double> e = ring_fd_errors(eb, {rc}, x, delta, hs);
        const double b = std::min(e[0], e[1]);
        CHECK(b <= 1e-6);
        worst = std::max(worst, b);
    }
    const std::vector<double> e = ring_fd_errors(eb, ring, x, delta, hs);
    const double b = std::min(e[0], e[1]);
    CHECK(b <= 1e-6);
    std::printf(
        "exact-band-integral-hessian-contact: %zu cells, Hessian vs central differences: worst "
        "cell %.1e, the ring %.1e\n",
        ring.size(),
        worst,
        b);
}

/**
 * Hidden check of the "exact" rule's one-vertex Hessian on a stored band:
 * EXACT_HESS_STATE=<band>.bin ./wmtk_test_topological_offset "[.hess_states]". The file is the
 * front-flux bench's (int32 nV, nF, nT; double delta; V; F; the band cells, 12 doubles each; the
 * band outside the input, sigma +1). For every front vertex (a vertex of a boundary face of the
 * band with d > delta / 2) the sum over its ring cells of value_grad_hess() against central
 * differences of that sum's gradient at steps h = 1e-3 .. 1e-7 delta; the error of a vertex is
 * the min over h. EXACT_HESS_VERTEX=<k>: per ring cell of the k-th front vertex, the same
 * errors, and the cell in hex floats.
 */
TEST_CASE("exact-band-hessian-states", "[.hess_states]")
{
    const char* file = std::getenv("EXACT_HESS_STATE");
    if (!file) {
        WARN("EXACT_HESS_STATE not set");
        return;
    }
    const BandState s = read_band_state(file);
    double vol6 = 0.;
    for (int i = 0; i < s.F.rows(); ++i) {
        const Vec3 a = s.V.row(s.F(i, 0)), b = s.V.row(s.F(i, 1)), c = s.V.row(s.F(i, 2));
        vol6 += a.dot(b.cross(c));
    }
    const ExactBandIntegral eb(
        s.V,
        s.F,
        std::vector<int8_t>(size_t(s.F.rows()), vol6 > 0. ? 1 : -1),
        1);
    const auto tris = triangle_envelope(s.V, s.F);
    const double pad = input_pad(s.V);
    const auto dist = [&](const Vec3& p) {
        return tris->primitive_distance(nearest_primitive(*tris, pad, p), p);
    };
    std::map<std::array<int, 3>, int> nface;
    std::vector<std::vector<int>> ring(s.P.size());
    for (size_t t = 0; t < s.cell.size(); ++t) {
        for (int i = 0; i < 4; ++i) {
            std::array<int, 3> k;
            int c = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != i) k[size_t(c++)] = s.cell[t][size_t(j)];
            }
            std::sort(k.begin(), k.end());
            ++nface[k];
            ring[size_t(s.cell[t][size_t(i)])].push_back(int(t));
        }
    }
    std::vector<char> on_boundary(s.P.size(), 0);
    for (const auto& [k, n] : nface) {
        if (n == 1)
            for (const int v : k) on_boundary[size_t(v)] = 1;
    }
    std::vector<int> front;
    for (size_t v = 0; v < s.P.size(); ++v) {
        if (on_boundary[v] && dist(s.P[v]) > 0.5 * s.delta) front.push_back(int(v));
    }
    const std::vector<double> hs =
        {1e-3 * s.delta, 1e-4 * s.delta, 1e-5 * s.delta, 1e-6 * s.delta, 1e-7 * s.delta};
    const auto ring_of = [&](const int v) {
        std::vector<RingCell> r;
        for (const int t : ring[size_t(v)]) {
            RingCell rc;
            for (int j = 0; j < 4; ++j) {
                rc.T[size_t(j)] = s.P[size_t(s.cell[size_t(t)][size_t(j)])];
                if (s.cell[size_t(t)][size_t(j)] == v) rc.kv = j;
            }
            r.push_back(rc);
        }
        return r;
    };
    const char* one = std::getenv("EXACT_HESS_VERTEX");
    if (one) {
        const int k = std::atoi(one);
        REQUIRE(k >= 0);
        REQUIRE(size_t(k) < front.size());
        const int v = front[size_t(k)];
        const std::vector<RingCell> r = ring_of(v);
        std::printf(
            "front vertex %d (id %d) x = (%a, %a, %a), d/delta %.6f, %zu ring cells, delta %a\n",
            k,
            v,
            s.P[size_t(v)][0],
            s.P[size_t(v)][1],
            s.P[size_t(v)][2],
            dist(s.P[size_t(v)]) / s.delta,
            r.size(),
            s.delta);
        for (size_t i = 0; i < r.size(); ++i) {
            double kink = 0.;
            const std::vector<double> e =
                ring_fd_errors(eb, {r[i]}, s.P[size_t(v)], s.delta, hs, &kink);
            Vec3 G;
            Eigen::Matrix3d H;
            ring_grad_hess(eb, {r[i]}, s.P[size_t(v)], s.delta, G, H);
            double dmin = 1e300;
            for (const Vec3& p : r[i].T) dmin = std::min(dmin, dist(p));
            ExactBandIntegral::Stats st;
            eb.value(r[i].T, s.delta, &st);
            std::printf(
                "  cell %zu kv %d |H| %.3e min corner d/delta %.3e pieces %lld rule_a %lld "
                "side_clipped %lld D %lld | err h=1e-3..1e-7 delta: %.1e %.1e %.1e %.1e %.1e kink "
                "%.1e\n   ",
                i,
                r[i].kv,
                H.norm(),
                dmin / s.delta,
                (long long)st.pieces,
                (long long)st.rule_a_pieces,
                (long long)st.side_clipped,
                (long long)st.side_D,
                e[0],
                e[1],
                e[2],
                e[3],
                e[4],
                kink);
            for (const Vec3& p : r[i].T) std::printf(" %a %a %a", p[0], p[1], p[2]);
            std::printf("\n");
        }
    }
    std::vector<double> best;
    int over = 0;
    for (size_t k = 0; k < front.size(); ++k) {
        const int v = front[k];
        double kink = 0.;
        const std::vector<double> e =
            ring_fd_errors(eb, ring_of(v), s.P[size_t(v)], s.delta, hs, &kink);
        const double b = *std::min_element(e.begin(), e.end());
        best.push_back(b);
        if (b > 1e-6) {
            ++over;
            std::printf(
                "  front vertex %zu (id %d): err h=1e-3..1e-7 delta: %.1e %.1e %.1e %.1e %.1e "
                "kink %.1e\n",
                k,
                v,
                e[0],
                e[1],
                e[2],
                e[3],
                e[4],
                kink);
        }
    }
    std::vector<double> sorted = best;
    std::sort(sorted.begin(), sorted.end());
    std::printf(
        "exact-band-hessian-states: %s: %zu band cells, %zu front vertices; Hessian vs central "
        "differences (min over h): median %.1e max %.1e; %d over 1e-6\n",
        file,
        s.cell.size(),
        front.size(),
        sorted.empty() ? 0. : sorted[sorted.size() / 2],
        sorted.empty() ? 0. : sorted.back(),
        over);
    CHECK(over == 0);
}
