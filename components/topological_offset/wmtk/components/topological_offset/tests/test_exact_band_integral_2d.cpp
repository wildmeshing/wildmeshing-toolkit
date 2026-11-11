// ExactBandIntegral2D against an independent reference: the midpoint rule over a fine uniform
// subdivision, d by brute force over the segments and points, the offset side by the closed
// polygon itself (t clipped by a fan triangulation of it, not the pseudonormal test the integral
// uses); and its derivatives against finite differences, as test_exact_band_integral.cpp's fd()
// does for 3D.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/topological_offset/ExactBandIntegral2D.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

using namespace wmtk::components::topological_offset;
using Eigen::Vector2d;
using Tri = std::array<Vector2d, 3>;

namespace {
double seg_dist(const Vector2d& x, const Vector2d& a, const Vector2d& b)
{
    const Vector2d ab = b - a;
    const double t = std::clamp((x - a).dot(ab) / ab.squaredNorm(), 0., 1.);
    return (x - (a + t * ab)).norm();
}

double cross(const Vector2d& a, const Vector2d& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

struct Input
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    std::vector<int> P;
    std::vector<int8_t> side;
    std::vector<Vector2d> polygon; ///< non-empty: a closed polygon, counter-clockwise
    /// a point strictly left of every edge of `polygon` (it is star-shaped from there): the
    /// triangles (kernel, p_i, p_i+1) tile it
    Vector2d kernel = Vector2d::Zero();
};

/// The midpoint rule for int over the triangle (a, b, c) of f: m^2 congruent sub-triangles, per
/// (i, j) an "up" one, and a "down" one where i + j <= m - 2.
template <typename F>
double midpoint_tri(const Vector2d& a, const Vector2d& b, const Vector2d& c, const int m, F&& f)
{
    const Vector2d e1 = (b - a) / double(m), e2 = (c - a) / double(m);
    const double w = 0.5 * std::abs(cross(e1, e2));
    double s = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; i + j < m; ++j) {
            const Vector2d p = a + double(i) * e1 + double(j) * e2;
            s += w * f(Vector2d(p + (e1 + e2) / 3.));
            if (i + j <= m - 2) s += w * f(Vector2d(p + 2. * (e1 + e2) / 3.));
        }
    }
    return s;
}

/// The convex polygon P (either orientation) cut to the left of the line a -> b (Sutherland-
/// Hodgman).
std::vector<Vector2d>
clip_left(const std::vector<Vector2d>& P, const Vector2d& a, const Vector2d& b)
{
    std::vector<Vector2d> out;
    for (size_t i = 0; i < P.size(); ++i) {
        const Vector2d& p = P[i];
        const Vector2d& q = P[(i + 1) % P.size()];
        const double sp = cross(b - a, p - a), sq = cross(b - a, q - a);
        if (sp >= 0.) out.push_back(p);
        if ((sp > 0. && sq < 0.) || (sp < 0. && sq > 0.))
            out.push_back(p + (sp / (sp - sq)) * (q - p));
    }
    return out;
}

/// int over t (and with sigma, over its part on the offset side of the closed polygon) of
/// d - delta, by the midpoint rule at sub-triangles of size about L / n, L t's longest edge. With
/// sigma, the inside is t clipped by each triangle of the polygon's fan from its kernel, each part
/// fanned again, so the integrand is continuous on every triangle the rule runs over, and the
/// outside is all of t minus the inside. Until 2026-11-11 the offset side was the winding number
/// of each sub-triangle's centroid, a jump of delta across the input inside sub-triangles, whose
/// error is O(h) and erratic. MEASURED (the L, band outside, r 0.02, trial 28, the one triangle
/// that failed, value - reference): with the winding number 3.06e-9, 1.86e-10, 1.28e-10 at n = 500
/// / 1000 / 2000, against a tolerance of 1.74e-9; with the clipped polygon 4.2e-13, 5.6e-14,
/// 2.0e-14, 4.2e-15, 8.2e-16 at n = 250 .. 4000 (band inside: 8.1e-13 .. 3.2e-15, 4x per
/// doubling). The value was right; the reference's side was not.
double reference(const Input& in, const int sigma, const Tri& t, const double delta, const int n)
{
    const auto f = [&](const Vector2d& x) {
        double m = std::numeric_limits<double>::infinity();
        for (Eigen::Index r = 0; r < in.E.rows(); ++r) {
            m = std::min(
                m,
                seg_dist(x, in.V.row(in.E(r, 0)).transpose(), in.V.row(in.E(r, 1)).transpose()));
        }
        for (const int p : in.P) m = std::min(m, (x - Vector2d(in.V.row(p).transpose())).norm());
        return m - delta;
    };
    double L = 0.;
    for (int i = 0; i < 3; ++i) L = std::max(L, (t[size_t(i)] - t[size_t((i + 1) % 3)]).norm());
    const double total = midpoint_tri(t[0], t[1], t[2], n, f);
    if (sigma == 0 || in.polygon.empty()) return total;
    double inside = 0.;
    const size_t np = in.polygon.size();
    for (size_t i = 0; i < np; ++i) {
        const Vector2d& p = in.polygon[i];
        const Vector2d& q = in.polygon[(i + 1) % np];
        std::vector<Vector2d> Q(t.begin(), t.end());
        Q = clip_left(Q, in.kernel, p);
        Q = clip_left(Q, p, q);
        Q = clip_left(Q, q, in.kernel);
        for (size_t j = 1; j + 1 < Q.size(); ++j) {
            double l = 0.;
            const Vector2d* c[3] = {&Q[0], &Q[j], &Q[j + 1]};
            for (int k = 0; k < 3; ++k) l = std::max(l, (*c[k] - *c[(k + 1) % 3]).norm());
            const int m = std::max(1, int(std::ceil(double(n) * l / L)));
            inside += midpoint_tri(Q[0], Q[j], Q[j + 1], m, f);
        }
    }
    return sigma < 0 ? inside : total - inside;
}

Input polyline(const std::vector<Vector2d>& pts, const bool closed)
{
    Input in;
    in.V.resize(Eigen::Index(pts.size()), 2);
    for (size_t i = 0; i < pts.size(); ++i) in.V.row(Eigen::Index(i)) = pts[i].transpose();
    const size_t ns = closed ? pts.size() : pts.size() - 1;
    in.E.resize(Eigen::Index(ns), 2);
    for (size_t i = 0; i < ns; ++i) {
        in.E(Eigen::Index(i), 0) = int(i);
        in.E(Eigen::Index(i), 1) = int((i + 1) % pts.size());
    }
    if (closed) {
        in.polygon = pts;
        in.side.assign(ns, 1); // counter-clockwise: (b - a) rotated clockwise points out
    }
    return in;
}

double area2(const Tri& t)
{
    return (t[1] - t[0]).x() * (t[2] - t[0]).y() - (t[1] - t[0]).y() * (t[2] - t[0]).x();
}
double longest(const Tri& t)
{
    double L = 0.;
    for (int i = 0; i < 3; ++i) L = std::max(L, (t[size_t(i)] - t[size_t((i + 1) % 3)]).norm());
    return L;
}

/// Random triangles of size r (corners within r of a centre), in both orientations, no thinner
/// than 2 area / L^2 = 0.02. The centre, by trial number mod 3: uniform over the input's
/// surroundings [-1.5, 1.5]^2; within r/3 of an input vertex (a corner, a reflex corner, a bend,
/// an isolated point); within r/3 of a point on a segment (the triangle across it).
struct Gen
{
    std::mt19937 rng{7};
    std::vector<Vector2d> verts;
    std::vector<std::pair<Vector2d, Vector2d>> segs;
    explicit Gen(const Input& in)
    {
        for (Eigen::Index r = 0; r < in.V.rows(); ++r) verts.push_back(in.V.row(r).transpose());
        for (Eigen::Index r = 0; r < in.E.rows(); ++r) {
            segs.push_back({in.V.row(in.E(r, 0)).transpose(), in.V.row(in.E(r, 1)).transpose()});
        }
    }
    Tri next(const int k, const double r)
    {
        std::uniform_real_distribution<double> U(-1., 1.), W(0., 1.);
        Vector2d c(1.5 * U(rng), 1.5 * U(rng));
        if (k % 3 == 1) {
            c = verts[size_t(W(rng) * double(verts.size())) % verts.size()];
        } else if (k % 3 == 2 && !segs.empty()) {
            const auto& [a, b] = segs[size_t(W(rng) * double(segs.size())) % segs.size()];
            c = a + W(rng) * (b - a);
        }
        if (k % 3 != 0) c += (r / 3.) * Vector2d(U(rng), U(rng));
        while (true) {
            Tri t;
            for (auto& x : t) x = c + r * Vector2d(U(rng), U(rng));
            const double L = longest(t);
            if (std::abs(area2(t)) >= 0.02 * L * L) return t;
        }
    }
};

/// value() against the reference on random triangles at the scales r. Exact where no rule-(a)
/// piece is taken; an upper bound (value >= reference - tol) where one is. Returns the worst
/// |value - reference| / tol over the exact triangles.
double
check(const char* name, const Input& in, const int sigma, const double delta, const int trials)
{
    const ExactBandIntegral2D J(in.V, in.E, in.P, in.side, sigma);
    Gen gen(in);
    double worst = 0., worst_bound = 0.;
    int n_exact = 0, n_bound = 0, n_clipped = 0;
    Tri t_worst;
    double v_worst = 0., r_worst = 0.;
    int k_worst = -1;
    for (const double r : {0.02, 0.2, 1.}) {
        for (int k = 0; k < trials; ++k) {
            const Tri t = gen.next(k, r);
            ExactBandIntegral2D::Stats st;
            const double v = J.value(t, delta, &st);
            const double ref = reference(in, sigma, t, delta, 500);
            const double area = st.area > 0. ? st.area : 1.;
            // the midpoint rule's error, O(h^2) with h = r / 500, well under 1e-4 of area x scale
            const double tol = 2e-5 * area * (1. + r);
            const double gap = std::abs(v - ref);
            n_clipped += st.side_clipped > 0;
            const bool ok = st.rule_a_area == 0. ? gap <= tol : v >= ref - tol;
            if (!ok) {
                // the reference's own convergence at the failing triangle: value - reference at n,
                // 2n, 4n (O(h^2): the gap falls 4x per doubling; O(h): 2x; flat: the integral)
                std::printf(
                    "  FAIL %s r %g trial %d: value - ref at n = 500 / 1000 / 2000: %.3e %.3e "
                    "%.3e, tol %.3e\n",
                    name,
                    r,
                    k,
                    v - ref,
                    v - reference(in, sigma, t, delta, 1000),
                    v - reference(in, sigma, t, delta, 2000),
                    tol);
            }
            INFO(
                name << " r " << r << " trial " << k << " value " << v << " reference " << ref
                     << " tol " << tol << " rule (a) area " << st.rule_a_area << " of " << st.area);
            if (st.rule_a_area == 0.) {
                ++n_exact;
                if (gap / tol >= worst) {
                    worst = gap / tol;
                    t_worst = t;
                    v_worst = v;
                    r_worst = r;
                    k_worst = k;
                }
                CHECK(gap <= tol);
            } else {
                ++n_bound;
                worst_bound = std::max(worst_bound, (ref - v) / tol);
                CHECK(v >= ref - tol); // rule (a): an upper bound
            }
        }
    }
    std::printf(
        "  %-22s %3d exact: max |value - ref| / tol %.3f | %3d with rule (a): max (ref - value) / "
        "tol %.3f | %d clipped\n",
        name,
        n_exact,
        worst,
        n_bound,
        worst_bound,
        n_clipped);
    if (k_worst >= 0) {
        // the reference's convergence at the worst exact triangle (O(h^2): the gap falls 4x per
        // doubling of n, until the value's own error or rounding)
        std::printf(
            "    worst (r %g trial %d): value - ref at n = 500 / 1000 / 2000: %.3e %.3e %.3e\n",
            r_worst,
            k_worst,
            v_worst - reference(in, sigma, t_worst, delta, 500),
            v_worst - reference(in, sigma, t_worst, delta, 1000),
            v_worst - reference(in, sigma, t_worst, delta, 2000));
    }
    return worst;
}

/// fd() of test_exact_band_integral.cpp in the plane: the errors of the gradient and Hessian
/// against central differences (gradient from value(), Hessian from value_grad_hess()) at each
/// step h L, h = 1e-3 .. 1e-7, L the longest edge, in units S / L and S / L^2, S = I + delta A,
/// I = int_t d dA; and at h = 1e-5 the kink measure |H_fwd - H_bwd| / |H| with the one-sided
/// error of H d along the inward direction d at h = 1e-6.
struct FD
{
    std::array<std::array<double, 2>, 5> rows; // per h: gradient, Hessian
    double kink = 0., inward6 = 0.;
};
FD fd(const ExactBandIntegral2D& eb, const Tri& T, const int kv, const double delta)
{
    constexpr double HS[5] = {1e-3, 1e-4, 1e-5, 1e-6, 1e-7};
    const double L = longest(T);
    const double S = eb.value(T, 0.) + delta * 0.5 * std::abs(area2(T));
    const double ug = S / L, uh = S / (L * L);
    Vector2d G;
    Eigen::Matrix2d H;
    eb.value_grad_hess(T, kv, delta, G, H);
    FD out;
    for (int ih = 0; ih < 5; ++ih) {
        const double hh = HS[ih] * L;
        Vector2d Gfd;
        Eigen::Matrix2d Hfd, Hf, Hb;
        for (int i = 0; i < 2; ++i) {
            double jv[2];
            Vector2d gv[2];
            for (int s = 0; s < 2; ++s) {
                Tri T2 = T;
                T2[size_t(kv)][i] += (s == 0 ? 1. : -1.) * hh;
                jv[s] = eb.value(T2, delta);
                Eigen::Matrix2d Hd;
                eb.value_grad_hess(T2, kv, delta, gv[s], Hd);
            }
            Gfd[i] = (jv[0] - jv[1]) / (2 * hh);
            Hfd.col(i) = (gv[0] - gv[1]) / (2 * hh);
            Hf.col(i) = (gv[0] - G) / hh;
            Hb.col(i) = (G - gv[1]) / hh;
        }
        out.rows[size_t(ih)] = {(Gfd - G).norm() / ug, (Hfd - H).norm() / uh};
        if (ih == 2) {
            out.kink = (Hf - Hb).norm() / std::max(H.norm(), 1e-300);
            Vector2d d = Vector2d::Zero();
            for (int i = 0; i < 3; ++i) {
                if (i != kv) d += T[size_t(i)] / 2.;
            }
            d = (d - T[size_t(kv)]).normalized();
            Tri T2 = T;
            T2[size_t(kv)] += 1e-6 * L * d;
            Vector2d G2;
            Eigen::Matrix2d H2;
            eb.value_grad_hess(T2, kv, delta, G2, H2);
            out.inward6 = ((G2 - G) / (1e-6 * L) - H * d).norm() / uh;
        }
    }
    return out;
}

/// test_exact_band_integral.cpp's KINK: |H_fwd - H_bwd| / |H| at h = 1e-5 above this: grad J has
/// a kink at x, and H is the one-sided limit for x moving into t.
constexpr double KINK = 1e-3;

/// fd() at every corner of random triangles at two scales: smooth corners' min over h, max over
/// corners, of the gradient and Hessian errors; kink corners' inward error. Also: J of
/// value_grad_hess() is value()'s and grad_hess() is value_grad_hess()'s, bit for bit.
void check_derivs(
    const char* name,
    const Input& in,
    const int sigma,
    const double delta,
    const int trials)
{
    const ExactBandIntegral2D eb(in.V, in.E, in.P, in.side, sigma);
    Gen gen(in);
    int smooth = 0, kinks = 0, rule_a = 0;
    double grad = 0., hess = 0., inward = 0.;
    for (const double r : {0.05, 0.4}) {
        for (int k = 0; k < trials; ++k) {
            const Tri t = gen.next(k, r);
            ExactBandIntegral2D::Stats st;
            const double v = eb.value(t, delta, &st);
            rule_a += st.rule_a_area > 0.;
            for (int kv = 0; kv < 3; ++kv) {
                Vector2d G, G2;
                Eigen::Matrix2d H, H2;
                CHECK(eb.value_grad_hess(t, kv, delta, G, H) == v);
                eb.grad_hess(t, kv, delta, G2, H2);
                CHECK(G2 == G);
                CHECK(H2 == H);
                const FD f = fd(eb, t, kv, delta);
                if (f.kink > KINK) {
                    ++kinks;
                    inward = std::max(inward, f.inward6);
                    continue;
                }
                ++smooth;
                double g = 1e300, h = 1e300;
                for (const auto& row : f.rows) g = std::min(g, row[0]), h = std::min(h, row[1]);
                INFO(
                    name << " r " << r << " trial " << k << " kv " << kv << " gradient " << g
                         << " Hessian " << h << " rule (a) area " << st.rule_a_area);
                CHECK(g <= 1e-7);
                CHECK(h <= 1e-4);
                grad = std::max(grad, g);
                hess = std::max(hess, h);
            }
        }
    }
    std::printf(
        "  %-22s %3d smooth corners: FD min over h, max: gradient %.1e Hessian %.1e | %d kink "
        "corners: inward %.1e | %d triangles with rule (a)\n",
        name,
        smooth,
        grad,
        hess,
        kinks,
        inward,
        rule_a);
    CHECK(inward <= 2e-2);
}

Input three_points()
{
    Input in;
    in.V.resize(3, 2);
    in.V << 0.1, 0.2, -0.6, -0.3, 0.8, -0.5;
    in.E.resize(0, 2);
    in.P = {0, 1, 2};
    return in;
}
Input open_polyline()
{
    return polyline({{-1., -0.4}, {-0.2, 0.3}, {0.4, -0.1}, {1.1, 0.5}}, false);
}
/// an L-shaped region, counter-clockwise: a reflex corner at (0, 0)
Input L_shape()
{
    Input in =
        polyline({{-0.6, -0.6}, {0.6, -0.6}, {0.6, 0.}, {0., 0.}, {0., 0.6}, {-0.6, 0.6}}, true);
    in.kernel = Vector2d(-0.3, -0.3);
    for (size_t i = 0; i < in.polygon.size(); ++i) {
        const Vector2d& p = in.polygon[i];
        const Vector2d& q = in.polygon[(i + 1) % in.polygon.size()];
        REQUIRE(cross(q - p, in.kernel - p) > 0.); // the reference's fan tiles the polygon
    }
    return in;
}
} // namespace

TEST_CASE("exact-band-integral-2d-segment", "[exact_band_integral_2d]")
{
    check("segment", polyline({{-0.5, 0.1}, {0.7, -0.2}}, false), 0, 0.3, 30);
}

TEST_CASE("exact-band-integral-2d-points", "[exact_band_integral_2d]")
{
    check("points", three_points(), 0, 0.4, 30);
}

TEST_CASE("exact-band-integral-2d-polyline", "[exact_band_integral_2d]")
{
    check("polyline", open_polyline(), 0, 0.3, 30);
}

TEST_CASE("exact-band-integral-2d-closed-sides", "[exact_band_integral_2d]")
{
    check("L, band outside", L_shape(), 1, 0.25, 30);
    check("L, band inside", L_shape(), -1, 0.25, 30);
    check("L, no side", L_shape(), 0, 0.25, 30);
}

TEST_CASE("exact-band-integral-2d-flat", "[exact_band_integral_2d]")
{
    const ExactBandIntegral2D eb = [] {
        const Input in = L_shape();
        return ExactBandIntegral2D(in.V, in.E, in.P, in.side, 1);
    }();
    const Vector2d p(0.3, -0.2), q(-0.5, 0.4);
    const std::vector<Tri> flats = {
        {p, q, Vector2d(0.3 * p + 0.7 * q)}, // 3 collinear
        {p, p, q}, // two coincident corners
        {p, p, p}, // all coincident
        {Vector2d(-0.6, -0.6), Vector2d(0., -0.6), Vector2d(0.6, -0.6)}, // on an input segment
    };
    for (const Tri& T : flats) {
        CHECK(eb.value(T, 0.) == 0.);
        CHECK(eb.value(T, 0.5) == 0.);
        Vector2d G;
        Eigen::Matrix2d H;
        CHECK(eb.value_grad_hess(T, 1, 0.5, G, H) == 0.);
        CHECK(G.isZero(0.));
        CHECK(H.isZero(0.));
    }
}

TEST_CASE("exact-band-integral-2d-derivatives", "[exact_band_integral_2d]")
{
    // The thresholds are 3D's exact-band-integral-derivatives' (gradient 1e-7, Hessian 1e-4,
    // kink corners' inward error 2e-2, in fd()'s units); this port's measured numbers are printed.
    std::printf("exact-band-integral-2d-derivatives\n");
    check_derivs("segment", polyline({{-0.5, 0.1}, {0.7, -0.2}}, false), 0, 0.3, 12);
    check_derivs("points", three_points(), 0, 0.4, 12);
    check_derivs("polyline", open_polyline(), 0, 0.3, 12);
    check_derivs("L, band outside", L_shape(), 1, 0.25, 12);
    check_derivs("L, band inside", L_shape(), -1, 0.25, 12);
}

TEST_CASE("exact-band-integral-2d-hessian-contact", "[exact_band_integral_2d]")
{
    // 3D's exact-band-integral-hessian-contact one dimension down: t's edge (x, o) runs through
    // (0.6, 0.6) in the L's notch, where the lines x = 0.6 and y = 0.6 (the ends of the two notch
    // edges' slabs) and y = x (the bisector of the notch's two edges, and of the vertices (0.6, 0)
    // and (0, 0.6)) meet. The pieces in the wedge {x >= 0.6, y >= 0.6} touch that edge at the one
    // point only, so only their contact terms (value_grad_hess()) pair with their neighbours' cut
    // terms there. d is continuous and its kink y = x crosses the edge transversally, so J is
    // smooth at x: the Hessian must match central differences at every corner, the kink measure
    // small. MEASURED: without the contact terms the Hessian at corners 0 and 1 came 0.82 off
    // (fd()'s units), with them 3.8e-11.
    const Input in = L_shape();
    const Tri t = {Vector2d(0.75, 0.45), Vector2d(0.45, 0.75), Vector2d(0.9, 0.9)};
    for (const int sigma : {0, 1}) {
        const ExactBandIntegral2D eb(in.V, in.E, in.P, in.side, sigma);
        for (int kv = 0; kv < 3; ++kv) {
            const FD f = fd(eb, t, kv, 0.25);
            double g = 1e300, h = 1e300;
            for (const auto& row : f.rows) g = std::min(g, row[0]), h = std::min(h, row[1]);
            std::printf(
                "  contact, sigma %d, corner %d: FD min over h: gradient %.1e Hessian %.1e, kink "
                "%.1e\n",
                sigma,
                kv,
                g,
                h,
                f.kink);
            CHECK(f.kink <= KINK);
            CHECK(g <= 1e-7);
            CHECK(h <= 1e-4);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// BandVolumeEnergy2D, the 2D front smoother's band term: the 2D twins of band-volume-exact-min
// (test_offset_potential.cpp) and exact-band-integral-energy (test_exact_band_integral.cpp), over
// a range of rings: around input vertices, across segments and anywhere near the input, at three
// scales, with and without a nearly flat face.
// ---------------------------------------------------------------------------------------------

namespace {

/// An open fan of faces (x0, q_i, q_i+1), i < n - 1, around x0: n = 4..7 arms at the angles
/// 2 pi i / n, each jittered by up to 0.3 of a gap (every gap below 145 degrees), radii in
/// [0.5, 1.5] r; or, with `needle`, four arms whose first gap is 179.8 degrees, so the face over
/// it is nearly flat (its height about 1.7e-3 of r). Open, as a front vertex's band faces are:
/// over a closed star of band faces the exact term does not move with x at all (its union is
/// fixed), nor the corner bound where one primitive's distance is affine on the star, and their
/// derivatives are rounding, which central differences cannot judge.
struct Star
{
    Vector2d x0;
    std::vector<BandVolumeEnergy2D::Cell> cells;
};
Star star_at(std::mt19937& rng, const Vector2d& c, const double r, const bool needle)
{
    std::uniform_real_distribution<double> U(0., 1.);
    const double pi = std::acos(-1.);
    const double base = 2. * pi * U(rng);
    std::vector<double> ang;
    if (needle) {
        ang = {0., 179.8, 250., 320.};
        for (double& a : ang) a = base + a * pi / 180.;
    } else {
        const int n = 4 + int(U(rng) * 4.) % 4;
        for (int i = 0; i < n; ++i)
            ang.push_back(base + 2. * pi * (i + 0.3 * (2. * U(rng) - 1.)) / n);
    }
    std::vector<Vector2d> q;
    for (const double a : ang) {
        const double rad = needle ? r : r * (0.5 + U(rng));
        q.push_back(c + rad * Vector2d(std::cos(a), std::sin(a)));
    }
    Star st;
    st.x0 = c;
    for (size_t i = 0; i + 1 < q.size(); ++i) st.cells.push_back({q[i], q[i + 1]});
    return st;
}

/// The shortest height of x over its faces' fixed edges: the length on which the term varies
/// with x, besides the distance from x to the input.
double min_height(const Vector2d& x, const std::vector<BandVolumeEnergy2D::Cell>& cells)
{
    double h = std::numeric_limits<double>::infinity();
    for (const auto& c : cells) {
        h = std::min(h, std::abs(cross(c.q1 - x, c.q2 - x)) / (c.q2 - c.q1).norm());
    }
    return h;
}

/// The corner bound of one face (TopoOffsetTriMesh::band_cell_corner_bound()'s arithmetic):
/// unsigned area times (min mean - delta) / delta.
double corner_rule(const ExactBandIntegral2D& eb, const Tri& t, const double delta)
{
    int64_t best = -1;
    const double md = eb.min_mean_distance(t.data(), 3, best);
    return std::abs(area2(t)) / 2. * ((md - delta) / delta);
}

/// `e`'s gradient g and Hessian H at x against central differences (the gradient from the value,
/// the Hessian from the gradient) at the steps h = hmin * 1e-2 .. 1e-6 that `accept` admits,
/// hmin the length on which the term varies: the errors |fd - g| and |fd - H| in fd()'s units
/// S / L and S / L^2 (S the term's scale, L the fan's longest edge), each the min over h, as fd()
/// takes them (the differences' truncation falls with h and their rounding grows, at a rate set by
/// value()'s own conditioning). Not relative to |g|: g is exactly 0 where the fan's edges through x
/// lie where the term's integrand is 0 (a band clipped by the offset side). And at h = 1e-4 hmin
/// the kink measure |H_fwd - H_bwd| / |H| (one-sided differences of the gradient), above 1e-3
/// where grad e has a kink at x.
struct EnergyFD
{
    double grad = 1e300, hess = 1e300, kink = 0.;
    Vector2d g, fd_g; ///< for the messages: g and its difference at the best step
};
template <typename Accept>
EnergyFD energy_fd(
    polysolve::nonlinear::Problem& e,
    const Vector2d& x,
    const double hmin,
    const double S,
    const double L,
    Accept&& accept)
{
    EnergyFD out;
    Eigen::VectorXd xv = x, g0;
    Eigen::MatrixXd H0;
    e.gradient(xv, g0);
    e.hessian(xv, H0);
    out.g = g0;
    for (const double f : {1e-2, 1e-3, 1e-4, 1e-5, 1e-6}) {
        const double h = f * hmin;
        if (!accept(h)) continue;
        Vector2d fg;
        Eigen::Matrix2d fH, Hf, Hb;
        for (int i = 0; i < 2; ++i) {
            Eigen::VectorXd xp = xv, xm = xv, gp, gm;
            xp[i] += h;
            xm[i] -= h;
            fg[i] = (e.value(xp) - e.value(xm)) / (2. * h);
            e.gradient(xp, gp);
            e.gradient(xm, gm);
            fH.col(i) = (gp - gm) / (2. * h);
            Hf.col(i) = (gp - g0) / h;
            Hb.col(i) = (g0 - gm) / h;
        }
        const double eg = (fg - Vector2d(g0)).norm() * L / S;
        if (eg < out.grad) out.grad = eg, out.fd_g = fg;
        out.hess = std::min(out.hess, (fH - Eigen::Matrix2d(H0)).norm() * L * L / S);
        if (f == 1e-4) out.kink = (Hf - Hb).norm() / std::max(fH.norm(), 1e-300);
    }
    return out;
}

/// The fan's longest edge, through x or not.
double fan_length(const Vector2d& x, const std::vector<BandVolumeEnergy2D::Cell>& cells)
{
    double L = 0.;
    for (const auto& c : cells) L = std::max(L, longest({{x, c.q1, c.q2}}));
    return L;
}

/// The position the derivatives are checked at: x0 moved by a tenth of the smaller of 0.1 r and
/// x0's shortest height over its faces, so every face stays valid (as the smoother keeps it).
Vector2d near_x0(std::mt19937& rng, const Star& st, const double r)
{
    std::uniform_real_distribution<double> U(-1., 1.);
    return st.x0 + 0.1 * std::min(0.1 * r, min_height(st.x0, st.cells)) * Vector2d(U(rng), U(rng));
}

} // namespace

TEST_CASE("band-area-exact-min-2d", "[exact_band_integral_2d]")
{
    // The 2D twin of band-volume-exact-min. ExactBandIntegral2D::min_mean_distance() is the min
    // over ALL primitives (segments, then points) of the corner mean of d_P (brute force over
    // primitive_distance(), lowest index on a tie), and primitive_distance() is the distance to
    // the segment or point with central-difference derivatives; the face term built on the min is
    // an upper bound on a dense integral and no split raises it; and BandVolumeEnergy2D has its
    // value and derivatives, over the range of rings above. The derivatives' thresholds are
    // fd()'s, 3D's exact-band-integral-derivatives' (gradient 1e-7, Hessian 1e-4, in units S / L
    // and S / L^2); the measured maxima are printed.
    const double delta = 0.25;
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> U(-1., 1.);
    Input mixed = open_polyline(); // segments and points: the polyline's, and three more points
    mixed.V.conservativeResize(mixed.V.rows() + 3, 2);
    mixed.V.bottomRows(3) << 0.3, 0.8, -0.7, -0.9, 0.9, -0.6;
    mixed.P = {int(mixed.V.rows()) - 3, int(mixed.V.rows()) - 2, int(mixed.V.rows()) - 1};
    const std::vector<std::pair<const char*, Input>> inputs = {
        {"L", L_shape()},
        {"points", three_points()},
        {"polyline and points", mixed}};

    SECTION("the min over all primitives, and primitive_distance()")
    {
        for (const auto& item : inputs) {
            const char* name = item.first;
            const Input& in = item.second; // a reference, not a binding: the lambda captures it
            INFO(name);
            const ExactBandIntegral2D eb(in.V, in.E, in.P);
            const int64_t n_prim = int64_t(in.E.rows()) + int64_t(in.P.size());
            const auto prim_ends = [&](const int64_t P) {
                if (P < int64_t(in.E.rows())) {
                    return std::pair<Vector2d, Vector2d>{
                        in.V.row(in.E(P, 0)).transpose(),
                        in.V.row(in.E(P, 1)).transpose()};
                }
                const Vector2d v = in.V.row(in.P[size_t(P - in.E.rows())]).transpose();
                return std::pair<Vector2d, Vector2d>{v, v};
            };
            for (int k = 0; k < 300; ++k) {
                const Vector2d c(1.4 * U(rng), 1.4 * U(rng));
                Tri q;
                for (auto& p : q) p = c + 0.3 * Vector2d(U(rng), U(rng));
                double want = std::numeric_limits<double>::infinity();
                int64_t wbest = -1;
                for (int64_t P = 0; P < n_prim; ++P) {
                    double sum = 0.;
                    for (const Vector2d& p : q) sum += eb.primitive_distance(P, p);
                    sum /= 3;
                    if (sum < want) want = sum, wbest = P;
                    const auto [a, b] = prim_ends(P);
                    CHECK(
                        eb.primitive_distance(P, q[0]) ==
                        Catch::Approx(a == b ? (q[0] - a).norm() : seg_dist(q[0], a, b))
                            .epsilon(1e-14));
                }
                int64_t best = -1;
                CHECK(eb.min_mean_distance(q.data(), 3, best) == want);
                CHECK(best == wbest);
            }
            // The derivatives in a segment's interior region, at an end and at a point.
            double worst_g = 0., worst_h = 0.;
            for (int k = 0; k < 200; ++k) {
                const Vector2d p(1.4 * U(rng), 1.4 * U(rng));
                const int64_t P = int64_t(k) % n_prim;
                const double d = eb.primitive_distance(P, p);
                if (!(d > 1e-3)) continue; // a kink at the primitive itself
                Vector2d g;
                Eigen::Matrix2d H;
                eb.primitive_distance(P, p, &g, &H);
                const double h = 1e-6;
                for (int j = 0; j < 2; ++j) {
                    Vector2d pp = p, pm = p, gp, gm;
                    pp[j] += h;
                    pm[j] -= h;
                    const double fd =
                        (eb.primitive_distance(P, pp) - eb.primitive_distance(P, pm)) / (2. * h);
                    worst_g = std::max(worst_g, std::abs(fd - g[j]));
                    eb.primitive_distance(P, pp, &gp);
                    eb.primitive_distance(P, pm, &gm);
                    worst_h = std::max(worst_h, ((gp - gm) / (2. * h) - H.col(j)).norm() * d);
                }
            }
            std::printf(
                "band-area-exact-min-2d, %s: primitive_distance() derivatives, max error: "
                "gradient %.1e, Hessian %.1e (times d)\n",
                name,
                worst_g,
                worst_h);
            CHECK(worst_g <= 1e-6);
            CHECK(worst_h <= 1e-5);
        }
    }
    SECTION("an upper bound, and no split raises it")
    {
        const Input in = L_shape();
        const ExactBandIntegral2D eb(in.V, in.E, in.P);
        Gen gen(in);
        int checked = 0;
        for (int k = 0; k < 60; ++k) {
            const Tri t = gen.next(k, 0.1);
            const double u = corner_rule(eb, t, delta);
            // the dense integral of (d - delta) / delta, no offset side (the corner bound has none)
            const double dense = reference(in, 0, t, delta, 200) / delta;
            const double tol = 1e-6 * std::abs(area2(t));
            INFO("trial " << k << " bound " << u << " dense " << dense);
            CHECK(u >= dense - tol);
            for (int ia = 0; ia < 3; ++ia) {
                const int ib = (ia + 1) % 3;
                const Vector2d mid = 0.5 * (t[size_t(ia)] + t[size_t(ib)]);
                Tri c1 = t, c2 = t;
                c1[size_t(ib)] = mid;
                c2[size_t(ia)] = mid;
                CHECK(
                    corner_rule(eb, c1, delta) + corner_rule(eb, c2, delta) <=
                    u + 1e-12 * std::max(1., std::abs(u)));
            }
            ++checked;
        }
        CHECK(checked == 60);
    }
    SECTION("BandVolumeEnergy2D: value and derivatives")
    {
        double worst_g = 0., worst_h = 0.;
        int n = 0, n_switch = 0;
        for (const auto& [name, in] : inputs) {
            const auto eb = std::make_shared<const ExactBandIntegral2D>(in.V, in.E, in.P);
            Gen gen(in);
            for (const double r : {1e-3, 0.03, 0.3}) {
                for (int k = 0; k < 24; ++k) {
                    const Tri t0 = gen.next(k, r); // a centre near a vertex, a segment or anywhere
                    const Star st = star_at(rng, t0[0], r, k % 4 == 3);
                    BandVolumeEnergy2D energy(eb, st.cells, st.x0, delta);
                    const Vector2d x = near_x0(rng, st, r);
                    const Eigen::VectorXd xv = x;
                    double want = 0.;
                    for (const auto& c : st.cells)
                        want += corner_rule(*eb, {{x, c.q1, c.q2}}, delta);
                    INFO(name << ", r " << r << ", ring " << k << ", x " << x.transpose());
                    CHECK(
                        energy.value(xv) ==
                        Catch::Approx(want).epsilon(1e-12).margin(1e-15 * r * r));
                    const double hmin =
                        std::min(min_height(x, st.cells), std::max(eb->distance(x), 1e-3 * r));
                    // the minimisers at x and x +- h: a switch puts a kink between them, and that
                    // step is not taken -- unless x's minimiser still attains the min there to
                    // rounding, a tie between primitives whose distances agree at all three
                    // corners (two segments ending at one input vertex, every corner in that
                    // vertex's region), which rounding breaks either way
                    int n_steps = 0;
                    const auto same = [&](const double h) {
                        for (const auto& c : st.cells) {
                            int64_t b0 = -1, b1 = -1;
                            const Tri tc = {{x, c.q1, c.q2}};
                            eb->min_mean_distance(tc.data(), 3, b0);
                            for (int i = 0; i < 2; ++i) {
                                for (const double sg : {-1., 1.}) {
                                    Tri ts = tc;
                                    ts[0][i] += sg * h;
                                    const double m1 = eb->min_mean_distance(ts.data(), 3, b1);
                                    if (b1 == b0) continue;
                                    double m0 = 0.;
                                    for (const Vector2d& p : ts)
                                        m0 += eb->primitive_distance(b0, p);
                                    if (!(m0 / 3. - m1 <= 1e-15 * m1)) return false;
                                }
                            }
                        }
                        ++n_steps;
                        return true;
                    };
                    // S = sum_t A_t (md_t + delta) / delta, the corner bound's twin of fd()'s
                    // I + delta A
                    double S = 0.;
                    for (const auto& c : st.cells) {
                        const Tri tc = {{x, c.q1, c.q2}};
                        int64_t best = -1;
                        S += std::abs(area2(tc)) / 2. *
                             ((eb->min_mean_distance(tc.data(), 3, best) + delta) / delta);
                    }
                    const EnergyFD f = energy_fd(energy, x, hmin, S, fan_length(x, st.cells), same);
                    if (f.grad == 1e300) {
                        ++n_switch;
                        continue;
                    }
                    INFO(
                        "gradient error " << f.grad << ", Hessian error " << f.hess << ", hmin "
                                          << hmin << " (" << n_steps << " steps), g "
                                          << f.g.transpose() << ", fd " << f.fd_g.transpose());
                    CHECK(f.grad <= 1e-7);
                    CHECK(f.hess <= 1e-4);
                    worst_g = std::max(worst_g, f.grad);
                    worst_h = std::max(worst_h, f.hess);
                    ++n;
                }
            }
        }
        std::printf(
            "band-area-exact-min-2d: BandVolumeEnergy2D corner bound, %d rings (%d skipped, a "
            "minimiser switching within h), max error (S / L, S / L^2): gradient %.1e Hessian "
            "%.1e\n",
            n,
            n_switch,
            worst_g,
            worst_h);
        CHECK(n > 150);
    }
}

TEST_CASE("exact-band-integral-energy-2d", "[exact_band_integral_2d]")
{
    // The 2D twin of exact-band-integral-energy: EXPERIMENTAL_band_volume_rule "exact" in the
    // smoother. BandVolumeEnergy2D's value is sum_t J_t / delta (ExactBandIntegral2D::value(), what
    // band_cell_term() reads), its gradient and Hessian sum_t of value_grad_hess() / delta in the
    // moving vertex x; equal bit for bit to those sums, whatever the order of the calls (each
    // keeps the last point's), and consistent with central differences -- over the range of rings
    // above, on a segment, on points and on the L with its offset side both ways. A ring with a
    // kink of grad J at x (|H_fwd - H_bwd| / |H| above 1e-3: a kink of d along an edge through x,
    // where H is the one-sided limit) is counted, its Hessian not checked. The derivatives'
    // thresholds are fd()'s, 3D's exact-band-integral-derivatives' (gradient 1e-7, Hessian 1e-4,
    // in units S / L and S / L^2); the measured maxima are printed.
    struct Case
    {
        const char* name;
        Input in;
        int sigma;
    };
    const std::vector<Case> cases = {
        {"segment", polyline({{-0.5, 0.1}, {0.7, -0.2}}, false), 0},
        {"points", three_points(), 0},
        {"L, band outside", L_shape(), 1},
        {"L, band inside", L_shape(), -1}};
    const double delta = 0.25;
    std::mt19937 rng(41);
    double worst_g = 0., worst_h = 0.;
    int n = 0, n_kink = 0;
    for (const Case& cs : cases) {
        const auto eb = std::make_shared<const ExactBandIntegral2D>(
            cs.in.V,
            cs.in.E,
            cs.in.P,
            cs.in.side,
            cs.sigma);
        Gen gen(cs.in);
        for (const double r : {1e-3, 0.03, 0.3}) {
            for (int k = 0; k < 12; ++k) {
                const Tri t0 = gen.next(k, r);
                const Star st = star_at(rng, t0[0], r, k % 4 == 3);
                BandVolumeEnergy2D energy(eb, st.cells, st.x0, delta);
                energy.set_exact_integral(eb, delta);
                const Vector2d x = near_x0(rng, st, r);
                const Eigen::VectorXd xv = x;
                double want = 0.;
                Vector2d gwant = Vector2d::Zero();
                Eigen::Matrix2d hwant = Eigen::Matrix2d::Zero();
                for (auto c : st.cells) {
                    // the constructor's orientation: q1, q2 swapped where the area at x0 is
                    // negative
                    if (cross(c.q1 - st.x0, c.q2 - st.x0) < 0.) std::swap(c.q1, c.q2);
                    want += eb->value({{x, c.q1, c.q2}}, delta) / delta;
                    Vector2d gc;
                    Eigen::Matrix2d hc;
                    eb->value_grad_hess({{x, c.q1, c.q2}}, 0, delta, gc, hc);
                    gwant += gc / delta;
                    hwant += hc / delta;
                }
                INFO(cs.name << ", r " << r << ", ring " << k << ", x " << x.transpose());
                Eigen::VectorXd g;
                Eigen::MatrixXd H;
                energy.gradient(xv, g); // derivatives first, then the value, then again
                CHECK(energy.value(xv) == want);
                energy.hessian(xv, H);
                CHECK(Vector2d(g) == gwant);
                CHECK(Eigen::Matrix2d(H) == hwant);
                CHECK(energy.value(xv) == want);
                const double hmin =
                    std::min(min_height(x, st.cells), std::max(eb->distance(x), 1e-3 * r));
                // S = sum_t (I_t + delta A_t) / delta, fd()'s units for E = sum_t J_t / delta
                double S = 0.;
                for (const auto& c : st.cells) {
                    const Tri tc = {{x, c.q1, c.q2}};
                    S += (eb->value(tc, 0.) + delta * std::abs(area2(tc)) / 2.) / delta;
                }
                const EnergyFD f =
                    energy_fd(energy, x, hmin, S, fan_length(x, st.cells), [](double) {
                        return true;
                    });
                INFO(
                    "gradient error " << f.grad << ", kink " << f.kink << ", hmin " << hmin
                                      << ", g " << f.g.transpose() << ", fd " << f.fd_g.transpose()
                                      << ", value " << want);
                CHECK(f.grad <= 1e-7);
                worst_g = std::max(worst_g, f.grad);
                if (f.kink > 1e-3) {
                    ++n_kink;
                    continue;
                }
                INFO("Hessian error " << f.hess);
                CHECK(f.hess <= 1e-4);
                worst_h = std::max(worst_h, f.hess);
                ++n;
            }
        }
    }
    std::printf(
        "exact-band-integral-energy-2d: %d rings (%d more with a kink at x), max error (S / L, "
        "S / L^2): "
        "gradient %.1e Hessian %.1e\n",
        n,
        n_kink,
        worst_g,
        worst_h);
    CHECK(n > 100);
}
