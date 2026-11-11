// ExactBandIntegral2D against an independent reference: the midpoint rule over a fine uniform
// subdivision, d by brute force over the segments and points, the offset side by the closed
// polygon itself (t clipped by a fan triangulation of it, not the pseudonormal test the integral
// uses); and its derivatives against finite differences, as test_exact_band_integral.cpp's fd()
// does for 3D.
#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/topological_offset/ExactBandIntegral2D.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
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
