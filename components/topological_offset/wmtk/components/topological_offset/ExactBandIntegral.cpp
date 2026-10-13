#include "ExactBandIntegral.hpp"

#include <wmtk/utils/Logger.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <numeric>
#include <string>

// Every function below is the Python function of the same name (exact_tri.py, exact_mesh.py,
// scratchpad exact_integral/), with its operation order kept: the reference's numbers are the
// spec, and the tests compare to them at rounding level.

namespace wmtk::components::topological_offset {

namespace {

using Vec3 = Eigen::Vector3d;
using Stats = ExactBandIntegral::Stats;

constexpr double EPS = std::numeric_limits<double>::epsilon();

// exact_tri.py, near-parallel branch. MEASURED there on 400 random faces, max relative error for
// eps_F in [0,1e-3) [1e-3,1e-2) [1e-2,0.1) [0.1,0.3) [0.3,1):
//   radial branch forced  3.1e-11  1.9e-12  1.3e-13  1.9e-14  7.8e-15
//   series branch forced  4.1e-15  5.4e-15  5.8e-15  4.6e-15  diverges (1.8e+01)
// so the switch sits at 0.1, with margin below the divergence. The long-edge binomial sums
// amplify rounding by about ((1 + 2/SHORT) eps_F / 2)^k, which must stay below 1.
constexpr double EPS_PAR = 0.1; ///< eps_F below which a face uses the near-parallel series
constexpr int K_LONG = 12; ///< truncation: (EPS_PAR / 2)^(K_LONG + 1) ~ 1e-17
constexpr double SHORT = 0.5; ///< edge "short" if its (eta, zeta) length <= SHORT * rho
constexpr int K_SHORT = 28; ///< even terms up to K_SHORT: (SHORT / 2)^(K_SHORT + 2) ~ 1e-18
// Each series stops at the actual ratio of its edge (short: q = |P2 - P1| / (2 rho) <= SHORT/2;
// long: |dz| / (2 zm) < EPS_PAR/2), at the order whose tail meets the same bound as the fixed
// orders above, which stay the cap (exact_tri._order_short / _order_long, adopted in both
// 2026-10-12). MEASURED (mac-mini): t100026 11.2 -> 6.8 us per cell, series edges 41% -> 14% of
// the time, J moved by <= 3.4e-14 relative, 99.9% of 22180 band cells bit-identical.

/// Highest even order kept for ratio q: q^(order + 2) <= (SHORT/2)^(K_SHORT + 2).
int order_short(const double q)
{
    const int k = q > 0. ? int(std::ceil((K_SHORT + 2) * std::log(SHORT / 2) / std::log(q))) - 2 : 0;
    return std::clamp(k + (k & 1), 0, K_SHORT);
}

/// Highest order kept for ratio r: r^(order + 1) <= (EPS_PAR/2)^(K_LONG + 1).
int order_long(const double r)
{
    const int k = r > 0. ? int(std::ceil((K_LONG + 1) * std::log(EPS_PAR / 2) / std::log(r))) - 1 : 0;
    return std::clamp(k, 0, K_LONG);
}

double maxabs(const Vec3& p)
{
    return p.cwiseAbs().maxCoeff();
}

// ------------------------------------------------------------------------------------------------
// Timing and counting (Stats); a null Stats costs one branch per call site.
// ------------------------------------------------------------------------------------------------

struct Ctx
{
    Stats* st = nullptr;
    bool timing = false;
    /// The partition's tolerance budget (ExactBandIntegral::partition(), the checks in value()):
    /// the sum of tol over the clips that cut (tol_cut), and over every decision taken with a
    /// tolerance, clips and region tests alike (tol_any). Null outside the partition.
    double* tol_cut = nullptr;
    double* tol_any = nullptr;
    /// The cell's rounding scale (ExactBandIntegral::partition()): every clip and region test of
    /// the cell takes its tolerance on it, never below the polytope's own coordinates.
    double scale = 0.;
};

struct Timer
{
    double* acc = nullptr;
    std::chrono::steady_clock::time_point t0;
    Timer(const Ctx& cx, double Stats::*field)
    {
        if (cx.timing) {
            acc = &(cx.st->*field);
            t0 = std::chrono::steady_clock::now();
        }
    }
    ~Timer()
    {
        if (acc) {
            *acc += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        }
    }
};

// ------------------------------------------------------------------------------------------------
// Convex polytopes (exact_tri: a list of faces (V, n), V counter-clockwise about the unit outward
// normal n). Face f is the ring P[off[f]] .. P[off[f + 1] - 1]. No faces = the Python's None.
// ------------------------------------------------------------------------------------------------

struct Poly
{
    std::vector<Vec3> P;
    std::vector<int> off{0};
    std::vector<Vec3> N;
    int faces() const { return int(N.size()); }
    int size(int f) const { return off[size_t(f) + 1] - off[size_t(f)]; }
    const Vec3* ring(int f) const { return P.data() + off[size_t(f)]; }
    void clear()
    {
        P.clear();
        off.assign(1, 0);
        N.clear();
    }
    void add_face(const Vec3* V, int k, const Vec3& n)
    {
        P.insert(P.end(), V, V + k);
        off.push_back(int(P.size()));
        N.push_back(n);
    }
    /// np.max(np.abs(vstack of the rings))
    double max_abs() const
    {
        double m = 0.;
        for (const Vec3& p : P) m = std::max(m, maxabs(p));
        return m;
    }
};

/// tet_polytope: the faces of T, outward; false if flat.
bool tet_polytope(const std::array<Vec3, 4>& T, Poly& out)
{
    double L = 0.;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < i; ++j) L = std::max(L, (T[size_t(i)] - T[size_t(j)]).norm());
    }
    const double vol6 = (T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]));
    if (L == 0. || std::abs(vol6) <= 64 * EPS * (L * L * L)) return false;
    out.clear();
    for (int i = 0; i < 4; ++i) {
        Vec3 V[3];
        int c = 0;
        for (int j = 0; j < 4; ++j) {
            if (j != i) V[c++] = T[size_t(j)];
        }
        Vec3 n = (V[1] - V[0]).cross(V[2] - V[0]);
        if (n.dot(V[0] - T[size_t(i)]) < 0) { // outward = away from the 4th vertex
            std::swap(V[0], V[2]);
            n = -n;
        }
        out.add_face(V, 3, n / n.norm());
    }
    return true;
}

std::pair<Vec3, Vec3> plane_basis(const Vec3& g)
{
    const Vec3 a = std::abs(g[0]) < 0.9 ? Vec3(1., 0., 0.) : Vec3(0., 1., 0.);
    Vec3 u = g.cross(a);
    u /= u.norm();
    return {u, g.cross(u)}; // u x w = g
}

/// _clean_ring: drop consecutive near-duplicate vertices; false if fewer than 3 remain.
bool clean_ring(const std::vector<Vec3>& P, const double tol, std::vector<Vec3>& Q)
{
    Q.clear();
    for (const Vec3& p : P) {
        if (Q.empty() || maxabs(p - Q.back()) > tol) Q.push_back(p);
    }
    while (Q.size() > 1 && maxabs(Q.front() - Q.back()) <= tol) Q.pop_back();
    return Q.size() >= 3;
}

enum class Cut { None, Same, New };

/// clip: Q intersected with {x : g.x <= h}, |g| = 1. Vertices within rounding distance of the
/// plane are on it. None: no interior; Same: Q entirely inside (the result IS Q, `out` untouched);
/// New: the result in `out`.
Cut clip(const Poly& Q, const Vec3& g, const double h, Poly& out, const Ctx& cx)
{
    Timer tm(cx, &Stats::t_clip);
    if (cx.st) ++cx.st->clips;
    const double tol = 16 * EPS * (std::abs(h) + std::max(Q.max_abs(), cx.scale));
    double smax = -std::numeric_limits<double>::infinity(), smin = -smax;
    for (const Vec3& p : Q.P) {
        const double s = p.dot(g) - h;
        smax = std::max(smax, s);
        smin = std::min(smin, s);
    }
    if (cx.tol_any) *cx.tol_any += tol;
    if (smax <= tol) return Cut::Same; // entirely inside
    if (smin >= -tol) return Cut::None; // entirely outside (or flat on the plane)
    out.clear();
    // per-thread scratch (clip is not re-entrant): MEASURED, its 7 temporaries per call were a
    // visible part of the cost
    thread_local std::vector<Vec3> cap, ring, cleaned, C;
    thread_local std::vector<double> s, ang;
    thread_local std::vector<int> ord;
    cap.clear();
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const int k = Q.size(f);
        s.resize(size_t(k));
        bool all_le = true, all_ge = true;
        for (int i = 0; i < k; ++i) {
            double si = V[i].dot(g) - h;
            if (std::abs(si) <= tol) si = 0.;
            s[size_t(i)] = si;
            all_le = all_le && si <= 0.;
            all_ge = all_ge && si >= 0.;
        }
        // A face of Q whose outward normal is g up to rounding lies in the cut plane: a convex
        // polytope has one supporting plane per outward normal, and the cap is the result's face
        // with normal g. So the face is part of the cap: its kept points go to the cap and it is
        // not kept itself (also when every vertex is within tol of the plane). Kept as well, it
        // counted twice in the volume. MEASURED: band cells of the sphere and the prism came out
        // 2.6e-6, 3.4% and 9% over their volume, with faces in the plane up to 1.3 tol (vertices
        // made at a larger scale, or by a coincident plane of another feature; normal equal to g
        // or 1 ulp off).
        const Vec3& nf = Q.N[size_t(f)];
        bool on_plane = true;
        for (int i = 0; i < k; ++i) on_plane = on_plane && s[size_t(i)] == 0.;
        const bool in_cap = nf.dot(g) > 0. && (on_plane || nf.cross(g).norm() <= 16 * EPS);
        if (all_le) {
            if (in_cap) {
                cap.insert(cap.end(), V, V + k);
            } else {
                out.add_face(V, k, nf);
                for (int i = 0; i < k; ++i) {
                    if (s[size_t(i)] == 0.) cap.push_back(V[i]);
                }
            }
            continue;
        }
        if (all_ge) {
            for (int i = 0; i < k; ++i) {
                if (s[size_t(i)] == 0.) cap.push_back(V[i]);
            }
            continue;
        }
        ring.clear();
        for (int i = 0; i < k; ++i) {
            const int j = (i + 1) % k;
            const double si = s[size_t(i)], sj = s[size_t(j)];
            if (si <= 0.) ring.push_back(V[i]);
            if (si == 0.) cap.push_back(V[i]);
            if ((si < 0. && 0. < sj) || (sj < 0. && 0. < si)) {
                const int a = si < 0. ? i : j, b = si < 0. ? j : i; // from the inside end
                const double sa = s[size_t(a)], sb = s[size_t(b)];
                const Vec3 x = V[a] + (sa / (sa - sb)) * (V[b] - V[a]);
                ring.push_back(x);
                cap.push_back(x);
            }
        }
        if (in_cap) {
            cap.insert(cap.end(), ring.begin(), ring.end());
            continue;
        }
        if (clean_ring(ring, tol, cleaned)) {
            out.add_face(cleaned.data(), int(cleaned.size()), Q.N[size_t(f)]);
        }
    }
    // _dedupe
    C.clear();
    for (const Vec3& p : cap) {
        bool keep = true;
        for (const Vec3& o : C) {
            if (!(maxabs(p - o) > tol)) {
                keep = false;
                break;
            }
        }
        if (keep) C.push_back(p);
    }
    if (C.size() >= 3) {
        const auto [u, w] = plane_basis(g);
        Vec3 c = Vec3::Zero();
        for (const Vec3& p : C) c += p;
        c /= double(C.size());
        ang.resize(C.size());
        for (size_t i = 0; i < C.size(); ++i) {
            ang[i] = std::atan2((C[i] - c).dot(w), (C[i] - c).dot(u));
        }
        ord.resize(C.size());
        std::iota(ord.begin(), ord.end(), 0);
        std::stable_sort(ord.begin(), ord.end(), [&](int a, int b) {
            return ang[size_t(a)] < ang[size_t(b)];
        });
        ring.clear();
        for (int i : ord) ring.push_back(C[size_t(i)]);
        out.add_face(ring.data(), int(ring.size()), g);
    }
    if (cx.tol_cut) *cx.tol_cut += tol;
    return out.faces() >= 4 ? Cut::New : Cut::None;
}

/// R = clip(R, g, h) in place; false (R emptied) if None.
bool clip_inplace(Poly& R, const Vec3& g, const double h, Poly& tmp, const Ctx& cx)
{
    const Cut c = clip(R, g, h, tmp, cx);
    if (c == Cut::None) {
        R.clear();
        return false;
    }
    if (c == Cut::New) std::swap(R, tmp);
    return true;
}

/// volume_centroid: volume and centroid of a convex polytope (fan of tets from one vertex).
std::pair<double, Vec3> volume_centroid(const Poly& Q)
{
    const Vec3 o = Q.P[0];
    double vol6 = 0.;
    Vec3 mom = Vec3::Zero();
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3 a = V[0] - o;
        for (int i = 1; i < Q.size(f) - 1; ++i) {
            const Vec3 b = V[i] - o, c = V[i + 1] - o;
            const double d = a.dot(b.cross(c));
            vol6 += d;
            mom += d * (a + b + c);
        }
    }
    if (vol6 <= 0.) return {0., o};
    return {vol6 / 6., o + mom / (4. * vol6)};
}

// ------------------------------------------------------------------------------------------------
// One-dimensional closed forms (exact_tri). Differences of antiderivatives in rationalized form
// where the two ends have the same sign, so a short or far interval does not cancel.
// ------------------------------------------------------------------------------------------------

/// int_{u1}^{u1+ell} sqrt(u^2 + k^2) du, ell > 0, k >= 0.
double int_sqrt(double u1, const double ell, const double k)
{
    double u2 = u1 + ell;
    if (k == 0.) return 0.5 * (u2 * std::abs(u2) - u1 * std::abs(u1));
    if (u1 * u2 <= 0.) { // interval contains u = 0: no cancellation
        const auto F = [k](double u) {
            return 0.5 * (u * std::hypot(u, k) + k * k * std::asinh(u / k));
        };
        return F(u2) - F(u1);
    }
    if (u2 < 0.) { // reflect to the positive side
        const double a = -u2, b = -u1;
        u1 = a;
        u2 = b;
    }
    const double R1 = std::hypot(u1, k), R2 = std::hypot(u2, k);
    const double A = ell * (u1 + u2) * (u1 * u1 + u2 * u2 + k * k) / (u2 * R2 + u1 * R1);
    const double B = std::asinh(ell * (u1 + u2) / (u2 * R1 + u1 * R2));
    return 0.5 * (A + k * k * B);
}

/// int over the segment [x1, x2] of the distance to the line {A + l e}.
double seg_line(const Vec3& x1, const Vec3& x2, const Vec3& A, const Vec3& e)
{
    const Vec3 d = x2 - x1;
    const double ell = d.norm();
    Vec3 w0 = x1 - A;
    w0 = w0 - w0.dot(e) * e; // perpendicular offset at x1
    Vec3 tp = d / ell;
    tp = tp - tp.dot(e) * e; // perpendicular part of the direction
    const double sa = tp.norm();
    if (sa <= EPS) return ell * w0.norm(); // segment parallel to the line
    const Vec3 t = tp / sa;
    const double beta = w0.dot(t);
    const double k = w0.cross(t).norm();
    return int_sqrt(beta, sa * ell, k) / sa;
}

/// int_0^eta sqrt(t^2 + b^2) dt.
double G_(const double eta, const double b)
{
    if (b == 0.) return 0.5 * eta * std::abs(eta);
    return 0.5 * (eta * std::hypot(eta, b) + b * b * std::asinh(eta / b));
}

/// Mean of G_(., b) over the interval between e1 and e2.
double mean_G(const double e1, const double e2, const double b)
{
    if (e1 <= 0. && e2 <= 0.) {
        if (e1 == 0. && e2 == 0.) return 0.;
        return -mean_G(-e1, -e2, b); // G is odd
    }
    if (e1 < 0. || e2 < 0.) { // opposite signs: split at 0
        const double lo = std::min(e1, e2), hi = std::max(e1, e2);
        return (hi * mean_G(0., hi, b) - (-lo) * mean_G(0., -lo, b)) / (hi - lo);
    }
    if (e1 == e2) return G_(e1, b);
    if (b == 0.) return (e1 * e1 + e1 * e2 + e2 * e2) / 6.;
    const double R1 = std::hypot(e1, b), R2 = std::hypot(e2, b);
    const double dR = (e1 + e2) / (R1 + R2); // (R2 - R1) / (e2 - e1)
    const double t1 = dR * (R2 * R2 + R1 * R2 + R1 * R1) / 6.;
    const double den = e2 * R1 + e1 * R2;
    const double x = (e2 - e1) * (e1 + e2) / den; // asinh(e2/b) - asinh(e1/b) = asinh(x)
    const double ash = (e1 + e2) / den * (x != 0. ? std::asinh(x) / x : 1.);
    return t1 + 0.5 * b * b * (e2 * ash + std::asinh(e1 / b) - dR);
}

// ------------------------------------------------------------------------------------------------
// Truncated power series ("jets"), exact_tri _jmul .. _jG. Length n <= JCAP.
// ------------------------------------------------------------------------------------------------

/// A jet's (or, in the derivative code, a polynomial's) capacity: the longest is seg_int()'s
/// short-segment series, K_SEG + 4 = 34 terms.
constexpr int JCAP = 48;
using Jet = std::array<double, JCAP>;

void jmul(const double* a, const double* b, const int n, double* c)
{
    for (int k = 0; k < n; ++k) {
        double s = 0.;
        for (int j = 0; j <= k; ++j) s += a[j] * b[k - j];
        c[k] = s;
    }
}

void jdiv(const double* a, const double* b, const int n, double* c)
{
    c[0] = a[0] / b[0];
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        for (int j = 1; j <= k; ++j) s += b[j] * c[k - j];
        c[k] = (a[k] - s) / b[0];
    }
}

void jsqrt(const double* a, const int n, double* c)
{
    c[0] = std::sqrt(a[0]);
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        for (int j = 1; j < k; ++j) s += c[j] * c[k - j];
        c[k] = (a[k] - s) / (2. * c[0]);
    }
}

/// Jets of asinh(u) and w = sqrt(1 + u^2).
void jasinh_w(const double* u, const int n, double* A, double* w)
{
    Jet t, du, q;
    jmul(u, u, n, t.data());
    t[0] += 1.;
    jsqrt(t.data(), n, w);
    for (int i = 0; i + 1 < n; ++i) du[size_t(i)] = double(i + 1) * u[i + 1];
    jdiv(du.data(), w, n - 1, q.data()); // d asinh(u) = du / w
    A[0] = std::asinh(u[0]);
    for (int i = 0; i + 1 < n; ++i) A[i + 1] = q[size_t(i)] / double(i + 1);
}

/// jmul(a, b) for a vanishing beyond its first 3 coefficients: the same sum in the same order,
/// the zero terms left out, so the same value.
void jmul3(const double* a, const double* b, const int n, double* c)
{
    for (int k = 0; k < n; ++k) {
        double s = 0.;
        s += a[0] * b[k];
        if (k >= 1) s += a[1] * b[k - 1];
        if (k >= 2) s += a[2] * b[k - 2];
        c[k] = s;
    }
}

/// jdiv(a, b) for b vanishing beyond its first 2 coefficients: the same sum, zero terms left out.
void jdiv2(const double* a, const double* b, const int n, double* c)
{
    c[0] = a[0] / b[0];
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        s += b[1] * c[k - 1];
        c[k] = (a[k] - s) / b[0];
    }
}

/// _jG for the two jets both series use, eta = e0 + e1 l and zeta = z0 + z1 l (linear; z0 > 0):
/// jets of G(eta, zeta) = int_0^eta sqrt(t^2 + zeta^2) dt and, if R is given, of
/// R = sqrt(eta^2 + zeta^2). The products with a linear factor skip its zero coefficients
/// (jmul3, jdiv2), which halves the dense jet products and changes no value (MEASURED: every
/// band cell of the five stored cases bit-identical to the dense _jG).
void jG_linear(
    const double e0,
    const double e1,
    const double z0,
    const double z1,
    const int n,
    double* G,
    double* R)
{
    Jet eta{}, zeta{}, u, A, w, z2, uw;
    eta[0] = e0;
    eta[1] = e1;
    zeta[0] = z0;
    zeta[1] = z1;
    jdiv2(eta.data(), zeta.data(), n, u.data());
    jasinh_w(u.data(), n, A.data(), w.data());
    jmul3(zeta.data(), zeta.data(), n, z2.data());
    jmul(u.data(), w.data(), n, uw.data());
    for (int i = 0; i < n; ++i) uw[size_t(i)] += A[size_t(i)];
    jmul3(z2.data(), uw.data(), n, G);
    for (int i = 0; i < n; ++i) G[i] *= 0.5;
    if (R) jmul3(zeta.data(), w.data(), n, R);
}

double binom(const int k, const int j)
{
    double c = 1.;
    for (int i = 1; i <= j; ++i) c = c * double(k - j + i) / double(i);
    return std::round(c);
}

/// _edge_near_parallel: int_{xi1}^{xi2} G(eta(xi), zeta(xi)) dxi for one edge, dxi = xi2 - xi1.
/// G is analytic in zeta about the edge's mid value zm with radius zm; the series below converge
/// with ratio <= SHORT/2 (short edges, along the edge) or < EPS_PAR/2 (long edges, in eta).
double edge_near_parallel(
    const double e1,
    const double e2,
    const double z1,
    const double z2,
    const double dxi,
    const Ctx& cx)
{
    Timer tm(cx, &Stats::t_series);
    const double em = 0.5 * (e1 + e2), zm = 0.5 * (z1 + z2);
    const double de = e2 - e1, dz = z2 - z1;
    const double rho = std::hypot(em, zm);
    if (std::hypot(de, dz) <= SHORT * rho) {
        // Taylor series of lambda -> G(P_m + lambda (P2 - P1)), mean over [-1/2, 1/2]
        if (cx.st) ++cx.st->series_short;
        const int K = order_short(std::hypot(de, dz) / (2. * rho)) + 1;
        Jet g;
        jG_linear(em, de, zm, dz, K, g.data(), nullptr); // R is not needed here
        double sum = 0., p = 1.;
        for (int i = 0; i < K; i += 2) {
            sum += g[size_t(i)] * p / double(i + 1);
            p *= 0.25; // 0.5^i, exact
        }
        return dxi * sum;
    }
    // long edge: exact in eta, Taylor in delta = zeta - zm = mu (eta - em)
    if (cx.st) ++cx.st->series_long;
    const int K = order_long(std::abs(dz) / (2. * zm)) + 1;
    const int KJ = K + 1; // jet length
    const double mu = dz / de;
    Jet zj{}, z2j;
    zj[0] = zm;
    zj[1] = 1.;
    jmul3(zj.data(), zj.data(), KJ, z2j.data());
    std::array<std::array<Jet, K_LONG + 1>, 2> Aend;
    for (int s = 0; s < 2; ++s) {
        const double ea = s == 0 ? e1 : e2;
        Jet G, R, R2, R3, t;
        jG_linear(ea, 0., zm, 1., KJ, G.data(), R.data());
        jmul(R.data(), R.data(), KJ, R2.data());
        jmul(R.data(), R2.data(), KJ, R3.data());
        std::array<Jet, K_LONG + 2> Phi; // Phi_m = int eta^m R d eta
        Phi[0] = G;
        for (int i = 0; i < KJ; ++i) Phi[1][size_t(i)] = R3[size_t(i)] / 3.;
        for (int m = 2; m <= K; ++m) {
            jmul3(z2j.data(), Phi[size_t(m - 2)].data(), KJ, t.data());
            const double em1 = std::pow(ea, double(m - 1));
            for (int i = 0; i < KJ; ++i) {
                Phi[size_t(m)][size_t(i)] =
                    (em1 * R3[size_t(i)] - double(m - 1) * t[size_t(i)]) / double(m + 2);
            }
        }
        // A_j = int eta^j G d eta = (eta^{j+1} G - Phi_{j+1}) / (j + 1)
        for (int jj = 0; jj < K; ++jj) {
            const double ep = std::pow(ea, double(jj + 1));
            for (int i = 0; i < KJ; ++i) {
                Aend[size_t(s)][size_t(jj)][size_t(i)] =
                    (ep * G[size_t(i)] - Phi[size_t(jj + 1)][size_t(i)]) / double(jj + 1);
            }
        }
    }
    double tot = 0.;
    for (int k = 0; k < K; ++k) {
        double sk = 0.;
        for (int jj = 0; jj <= k; ++jj) {
            sk += binom(k, jj) * std::pow(-em, double(k - jj)) *
                  (Aend[1][size_t(jj)][size_t(k)] - Aend[0][size_t(jj)][size_t(k)]);
        }
        tot += std::pow(mu, double(k)) * sk;
    }
    return dxi / de * tot;
}

// ------------------------------------------------------------------------------------------------
// Face integrals over a planar convex polygon V[0..k), CCW about the unit normal n (exact_tri).
// _edges skips zero-length edges and gives t = unit direction, m = t x n (in-plane outward).
// ------------------------------------------------------------------------------------------------

/// int_F |x - v| dA.
double face_point(const Vec3* V, const int k, const Vec3& n, const Vec3& v, const Ctx& cx)
{
    Timer tm(cx, &Stats::t_closed);
    if (cx.st) ++cx.st->point_faces;
    const double z0 = n.dot(V[0] - v);
    const double c = std::abs(z0);
    const Vec3 q = v + z0 * n; // foot of v on the face plane
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m); // signed distance from q to the edge line
        if (h == 0.) continue;
        if (cx.st) ++cx.st->closed_edges;
        const double s1 = (x1 - q).dot(t);
        const double s2 = s1 + ell;
        const double a = std::hypot(h, c);
        double term = h * int_sqrt(s1, ell, a);
        if (c > 0.) {
            const double R1 = std::hypot(s1, a), R2 = std::hypot(s2, a);
            double num, ash;
            if (s1 * s2 > 0.) {
                const double den = s2 * R1 + s1 * R2;
                num = a * a * ell * (s1 + s2) / den; // s2 R1 - s1 R2
                ash = std::asinh(ell * (s1 + s2) / den); // asinh(s2/a) - asinh(s1/a)
            } else {
                num = s2 * R1 - s1 * R2;
                ash = std::asinh(s2 / a) - std::asinh(s1 / a);
            }
            // c^3 [atan(s c/(h R)) - atan(s/h)] between the ends, as exact angle differences
            const double ang = std::atan2(c * num * h, h * h * R1 * R2 + s1 * s2 * c * c) -
                               std::atan2(ell * h, h * h + s1 * s2);
            term += h * c * c * ash + c * c * c * ang;
        }
        tot += term;
    }
    return tot / 3.;
}

/// _face_line_parallel: line exactly parallel to the plane, r = sqrt(eta^2 + b^2), field (G, 0).
double face_line_parallel(const Vec3* V, const int k, const Vec3& n, const Vec3& A, const Vec3& e,
                          const Ctx& cx)
{
    if (cx.st) ++cx.st->line_faces_parallel;
    const Vec3 eh = n.cross(e); // in-plane, across the projected line
    const double b = std::abs(n.dot(V[0] - A));
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3& x2 = V[(i + 1) % k];
        const Vec3 d = x2 - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 m = (d / ell).cross(n);
        const double mh = m.dot(eh);
        if (mh != 0.) {
            if (cx.st) ++cx.st->closed_edges;
            tot += mh * ell * mean_G((x1 - A).dot(eh), (x2 - A).dot(eh), b);
        }
    }
    return tot;
}

/// int_F r dA, r = distance to the line {A + l e}. Three branches (exact_tri.face_line): exactly
/// parallel (closed form), near-parallel eps_F < EPS_PAR (series), otherwise radial about the
/// crossing point q, 1/3 sum_e h_e int_e r ds.
double face_line(const Vec3* V, const int k, const Vec3& n, const Vec3& A, const Vec3& e,
                 const Ctx& cx)
{
    Timer tm(cx, &Stats::t_closed);
    const double sn = e.dot(n);
    if (sn == 0.) return face_line_parallel(V, k, n, A, e, cx);
    if (std::abs(sn) < 0.5) { // in-plane frame well defined; else q is near
        const Vec3 ep = e - sn * n;
        const double cth = ep.norm();
        const Vec3 exi = ep / cth;
        const Vec3 eeta = n.cross(exi); // (exi, eeta, n) right-handed
        Vec3 vm = Vec3::Zero();
        for (int i = 0; i < k; ++i) vm += V[i];
        vm /= double(k);
        const Vec3 Ac = A + (vm - A).dot(e) * e;
        const double H = n.dot(V[0] - Ac);
        const auto xi_of = [&](int i) { return (V[i] - Ac).dot(exi); };
        const auto eta_of = [&](int i) { return (V[i] - Ac).dot(eeta); };
        const auto zeta_of = [&](double xi) { return xi * sn - H * cth; };
        double ximin = std::numeric_limits<double>::infinity(), ximax = -ximin;
        double zmin = ximin, zmax = -ximin;
        for (int i = 0; i < k; ++i) {
            const double xi = xi_of(i), z = zeta_of(xi);
            ximin = std::min(ximin, xi);
            ximax = std::max(ximax, xi);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        if (zmin > 0. || zmax < 0.) {
            const double sg = zeta_of(xi_of(0)) > 0 ? 1. : -1.;
            const double zamin = sg > 0 ? zmin : -zmax; // min of sg * zeta, G is even in zeta
            if (std::abs(sn) * (ximax - ximin) < EPS_PAR * zamin) {
                if (cx.st) ++cx.st->line_faces_series;
                double tot = 0.;
                for (int i = 0; i < k; ++i) {
                    const int j = (i + 1) % k;
                    const double xii = xi_of(i), xij = xi_of(j);
                    if (xij != xii) {
                        tot -= edge_near_parallel(
                            eta_of(i),
                            eta_of(j),
                            sg * zeta_of(xii),
                            sg * zeta_of(xij),
                            xij - xii,
                            cx);
                    }
                }
                return tot;
            }
        }
    }
    if (cx.st) ++cx.st->line_faces_radial;
    const Vec3 q = A + (n.dot(V[0] - A) / sn) * e; // line meets the face plane at q
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3& x2 = V[(i + 1) % k];
        const Vec3 d = x2 - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 m = (d / ell).cross(n);
        const double h = (x1 - q).dot(m);
        if (h != 0.) {
            if (cx.st) ++cx.st->closed_edges;
            tot += h * seg_line(x1, x2, A, e);
        }
    }
    return tot / 3.;
}

// ------------------------------------------------------------------------------------------------
// Volume integrals over a convex polytope (exact_tri), Euler's theorem: int_Q f = 1/4 sum_F h_F
// int_F f for f homogeneous of degree 1 about o; a face through o (h_F = 0) drops out.
// ------------------------------------------------------------------------------------------------

/// _through: the face plane passes through o up to rounding; its Euler term is rounding noise.
bool through(const double h, const Vec3* V, const int k, const Vec3& o)
{
    double m = 0.;
    for (int i = 0; i < k; ++i) m = std::max(m, maxabs(V[i]));
    return std::abs(h) <= 16 * EPS * (m + maxabs(o));
}

/// int_Q |x - v| dV.
double vol_point(const Poly& Q, const Vec3& v, const Ctx& cx)
{
    double tot = 0.;
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3& n = Q.N[size_t(f)];
        const double h = n.dot(V[0] - v);
        if (!through(h, V, Q.size(f), v)) tot += h * face_point(V, Q.size(f), n, v, cx);
    }
    return tot / 4.;
}

/// int_Q r dV, Euler centre o on the line, nearest the mean of the ring vertices.
double vol_line(const Poly& Q, const Vec3& A, const Vec3& e, const Ctx& cx)
{
    Vec3 c = Vec3::Zero();
    for (const Vec3& p : Q.P) c += p;
    c /= double(Q.P.size());
    const Vec3 o = A + (c - A).dot(e) * e; // line point nearest the piece
    double tot = 0.;
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3& n = Q.N[size_t(f)];
        const double h = n.dot(V[0] - o);
        if (!through(h, V, Q.size(f), o)) tot += h * face_line(V, Q.size(f), n, o, e, cx);
    }
    return tot / 4.;
}

/// int_Q |n.(x - a0)| dV: split by the plane, affine on each side.
double vol_plane(const Poly& Q, const Vec3& n, const Vec3& a0, const Ctx& cx)
{
    double tot = 0.;
    Poly tmp;
    for (const double sgn : {1., -1.}) { // keep sgn * n.(x - a0) >= 0
        const Cut c = clip(Q, -sgn * n, -sgn * n.dot(a0), tmp, cx);
        if (c == Cut::None) continue;
        const auto [vol, cen] = volume_centroid(c == Cut::Same ? Q : tmp);
        tot += vol * sgn * n.dot(cen - a0);
    }
    return tot;
}

// ------------------------------------------------------------------------------------------------
// exact_mesh: features in local coordinates, pieces, member sets.
// ------------------------------------------------------------------------------------------------

/// A candidate feature in the cell's local coordinates (origin c0 = the cell's centroid).
struct Loc
{
    int kind; // 0 face, 1 edge, 2 vertex
    Vec3 p, dir; // face: (a0, n); edge: (a, e); vertex: (v, -)
    int h_begin, h_end; // into the local half-spaces
};

double int_g(const Loc& g, const Poly& Q, const Ctx& cx)
{
    if (g.kind == 0) return vol_plane(Q, g.dir, g.p, cx);
    if (g.kind == 1) return vol_line(Q, g.p, g.dir, cx);
    return vol_point(Q, g.p, cx);
}

/// _d_feat: d_g at one point.
double d_feat(const Loc& g, const Vec3& y)
{
    if (g.kind == 0) return std::abs((y - g.p).dot(g.dir));
    if (g.kind == 1) {
        const Vec3 w = y - g.p;
        return (w - w.dot(g.dir) * g.dir).norm();
    }
    return (y - g.p).norm();
}

/// _bounds: (lower, upper) bound of d_g over the convex hull of the points Vq. Upper: the max at
/// the points (d_g convex). Lower: exact for a plane (affine per side), else d_g(c) - max |x_i - c|
/// (1-Lipschitz).
std::pair<double, double> bounds(const Loc& g, const std::vector<Vec3>& Vq)
{
    double up = 0.;
    for (const Vec3& y : Vq) up = std::max(up, d_feat(g, y));
    double lo;
    if (g.kind == 0) {
        double smin = std::numeric_limits<double>::infinity(), smax = -smin, amin = smin;
        for (const Vec3& y : Vq) {
            const double s = (y - g.p).dot(g.dir);
            smin = std::min(smin, s);
            smax = std::max(smax, s);
            amin = std::min(amin, std::abs(s));
        }
        lo = (smin < 0. && 0. < smax) ? 0. : amin;
    } else {
        Vec3 c = Vec3::Zero();
        for (const Vec3& y : Vq) c += y;
        c /= double(Vq.size());
        double r = 0.;
        for (const Vec3& y : Vq) r = std::max(r, (y - c).norm());
        lo = std::max(0., d_feat(g, c) - r);
    }
    return {lo, up};
}

enum class Side { In, Out, Split };

/// _classify: 'in' if the piece lies in all half-spaces, 'out' if outside one, else 'split'.
Side classify(
    const Poly& Q,
    const std::pair<Vec3, double>* H,
    const int nh,
    double* tol_any,
    const double scale)
{
    const double ma = std::max(Q.max_abs(), scale);
    bool s_in = true;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        double smin = std::numeric_limits<double>::infinity(), smax = -smin;
        for (const Vec3& p : Q.P) {
            const double s = p.dot(g) - h;
            smin = std::min(smin, s);
            smax = std::max(smax, s);
        }
        const double tol = 16 * EPS * (std::abs(h) + ma);
        if (tol_any) *tol_any += tol;
        if (smin >= -tol) return Side::Out;
        if (smax > tol) s_in = false;
    }
    return s_in ? Side::In : Side::Split;
}

/// _split: Q inside all of H (R; emptied if None), and the convex parts of Q outside.
void split(
    const Poly& Q,
    const std::pair<Vec3, double>* H,
    const int nh,
    Poly& R,
    std::vector<Poly>& outside,
    const Ctx& cx)
{
    R = Q;
    Poly tmp;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        const Cut c = clip(R, -g, -h, tmp, cx);
        if (c == Cut::Same) outside.push_back(R);
        if (c == Cut::New) outside.push_back(std::move(tmp));
        if (!clip_inplace(R, g, h, tmp, cx)) break;
    }
}

/// _clip_affine: Q intersected with {w.x <= rhs}, in place; false if empty.
bool clip_affine(Poly& R, const Vec3& w, const double rhs, Poly& tmp, const Ctx& cx)
{
    const double nw = w.norm();
    return clip_inplace(R, w / nw, rhs / nw, tmp, cx);
}

/// _faces_min_pieces: split Q so that min_i |n_i.(x - a_i)| is one affine function W.x - c per
/// sub-piece (split by the planes, each |.| then affine, then by the bisectors of the affine
/// distances); `piece(R, W, c)` for each sub-piece, in the Python's order.
template <typename F>
void faces_min_pieces(const Poly& Q, const std::vector<const Loc*>& faces, const Ctx& cx, F&& piece)
{
    struct Cell
    {
        Poly C;
        std::vector<double> sg;
    };
    std::vector<Cell> cells(1);
    cells[0].C = Q;
    Poly tmp;
    for (const Loc* f : faces) {
        const Vec3& n = f->dir;
        const Vec3& a0 = f->p;
        std::vector<Cell> next;
        for (const Cell& cl : cells) {
            for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
                const Cut c = clip(cl.C, -s * n, -s * n.dot(a0), tmp, cx);
                if (c == Cut::None) continue;
                Cell nc;
                nc.C = c == Cut::Same ? cl.C : std::move(tmp);
                nc.sg = cl.sg;
                nc.sg.push_back(s);
                next.push_back(std::move(nc));
            }
        }
        cells = std::move(next);
    }
    const size_t nf = faces.size();
    std::vector<Vec3> W(nf);
    std::vector<double> cc(nf);
    Poly R;
    for (const Cell& cl : cells) {
        for (size_t i = 0; i < nf; ++i) { // d_i = W_i . x - c_i
            W[i] = cl.sg[i] * faces[i]->dir;
            cc[i] = cl.sg[i] * faces[i]->dir.dot(faces[i]->p);
        }
        for (size_t i = 0; i < nf; ++i) {
            R = cl.C;
            bool alive = true;
            for (size_t j = 0; j < nf && alive; ++j) {
                if (j == i) continue;
                const Vec3 w = W[i] - W[j];
                const double rhs = cc[i] - cc[j];
                if (w.norm() <= 1e-14) { // identical function: lower index wins
                    if (j < i) alive = false;
                    continue;
                }
                alive = clip_affine(R, w, rhs, tmp, cx);
            }
            if (alive) piece(R, W[i], cc[i]);
        }
    }
}

/// _faces_min: exact int_Q min_i |n_i.(x - a_i)| dV.
double faces_min(const Poly& Q, const std::vector<const Loc*>& faces, const Ctx& cx)
{
    double tot = 0.;
    faces_min_pieces(Q, faces, cx, [&](const Poly& R, const Vec3& W, const double c) {
        const auto [vol, cen] = volume_centroid(R);
        tot += vol * (W.dot(cen) - c);
    });
    return tot;
}

/// _vertices_min_pieces: split Q by the bisector planes of the points; `piece(R, v)` for each
/// sub-piece with its nearest vertex v.
template <typename F>
void vertices_min_pieces(const Poly& Q, const std::vector<const Loc*>& verts, const Ctx& cx, F&& piece)
{
    Poly R, tmp;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3& vi = verts[i]->p;
        R = Q;
        bool alive = true;
        for (size_t j = 0; j < verts.size() && alive; ++j) {
            if (j == i) continue;
            const Vec3& vj = verts[j]->p;
            alive = clip_affine(R, 2. * (vj - vi), vj.dot(vj) - vi.dot(vi), tmp, cx);
        }
        if (alive) piece(R, vi);
    }
}

/// _vertices_min: exact int_Q min_i |x - v_i| dV (the bisectors of points are planes).
double vertices_min(const Poly& Q, const std::vector<const Loc*>& verts, const Ctx& cx)
{
    double tot = 0.;
    vertices_min_pieces(Q, verts, cx, [&](const Poly& R, const Vec3& v) { tot += vol_point(R, v, cx); });
    return tot;
}

/// point_tri_dist for one triangle: min over the three segments, and the plane distance when the
/// projection is inside.
double point_tri_dist(const Vec3& p, const Vec3& A, const Vec3& B, const Vec3& C)
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [P, Q] : {std::pair{&A, &B}, std::pair{&B, &C}, std::pair{&C, &A}}) {
        const Vec3 d = *Q - *P;
        const double s = std::clamp((p - *P).dot(d) / d.dot(d), 0., 1.);
        best = std::min(best, (p - *P - s * d).norm());
    }
    const Vec3 v0 = B - A, v1 = C - A, w = p - A;
    const double d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1);
    const double d20 = w.dot(v0), d21 = w.dot(v1);
    const double den = d00 * d11 - d01 * d01;
    const double l1 = (d11 * d20 - d01 * d21) / den;
    const double l2 = (d00 * d21 - d01 * d20) / den;
    if (l1 >= 0 && l2 >= 0 && l1 + l2 <= 1) {
        const Vec3 n = v0.cross(v1);
        best = std::min(best, std::abs(w.dot(n)) / n.norm());
    }
    return best;
}

/// box_dist: distance between the box [lo, hi] and t's box [tlo, thi].
double box_dist(const Vec3& lo, const Vec3& hi, const Vec3& tlo, const Vec3& thi)
{
    double s = 0.;
    for (int k = 0; k < 3; ++k) {
        const double g = std::max(0., std::max(lo[k] - thi[k], tlo[k] - hi[k]));
        s += g * g;
    }
    return std::sqrt(s);
}

struct Piece
{
    Poly Q;
    std::vector<int> S; // member candidates
};

/// The cell's corners and delta in hex floats, bit exact, for the error messages: a failing cell
/// can be replayed offline.
std::string cell_string(const std::array<Vec3, 4>& T, const double delta)
{
    std::string s = "t =";
    char buf[128];
    for (const Vec3& p : T) {
        std::snprintf(buf, sizeof(buf), " (%a, %a, %a)", p[0], p[1], p[2]);
        s += buf;
    }
    std::snprintf(buf, sizeof(buf), ", delta = %a", delta);
    return s + buf;
}

/// _prune_members: drop g if its lower bound on the piece is >= another member's upper bound (it
/// cannot attain the minimum there); changes no value.
std::vector<int> prune_members(const Poly& Q, const std::vector<int>& S, const std::vector<Loc>& loc)
{
    std::vector<std::pair<double, double>> b;
    b.reserve(S.size());
    for (const int k : S) b.push_back(bounds(loc[size_t(k)], Q.P));
    double ubmin = std::numeric_limits<double>::infinity();
    for (const auto& lu : b) ubmin = std::min(ubmin, lu.second);
    std::vector<int> kept;
    for (size_t i = 0; i < S.size(); ++i) {
        if (b[i].first < ubmin || b[i].second == ubmin) kept.push_back(S[i]);
    }
    return kept;
}

// ================================================================================================
// Exact gradient and Hessian with respect to one vertex x of t (exact_mesh.py, "Exact gradient and
// Hessian"; every function below is the Python one of the same name).
//
// J(x) = int_{t(x)} (d - delta) dV. Every point of t moves with velocity lambda0(y) per unit motion
// of x (lambda0 the barycentric coordinate of x), so by Reynolds' transport theorem, for a piece
// Q(x) = t(x) cap (fixed region) with a smooth integrand phi on Q:
//   grad int_Q phi = sum_F nu_F int_Pi phi lambda0 dA,
// F the three faces of t through x, Pi = Q cap F, nu_F the outward unit normal; and
//   hess int_Q phi = sum_F [ nu_F (x) int_Pi lambda0^2 grad phi dA + S_F int_Pi phi lambda0 dA / |N_F|
//                            - sum_{cut edges c of Pi} nu_F (x) g_c/|g_c,Pi| int_c phi lambda0^2 ds ],
// S_F the matrix of v -> v x (a - b) for F = {alpha x + beta a + gamma b}, N_F = (x - b) x (a - b)
// outward, and a cut edge an edge of Pi on a fixed plane with outward normal g_c: F tilts under
// that plane, so the edge slides over the material points of F. Summed over the pieces, the cut
// terms of two pieces sharing an edge cancel when phi is continuous across it (every exact
// piece); they remain on rule-(a) pieces, whose bound is not. A kink of d lying ON a face of t
// makes J one-sided C^2: the trace of grad phi from inside t is used, i.e. the Hessian for x
// moving into t.
// ================================================================================================

constexpr int K_SEG = 30; ///< Taylor order on short segments (ratio <= SHORT/2 = 1/4)
// Derivative integrals carry polynomial weights, which make the radial branch lose up to ~1e-12 at
// eps_F ~ 0.1-0.4 (Python, measured: radial forced 1.3e-9 max, default 1.5e-12); the series branch
// is therefore used up to EPS_PAR_D, with more terms: per-edge ratio <= EPS_PAR_D/2 = 0.125,
// 0.125^19 ~ 1e-17; rounding growth ((1 + 2/SHORT) EPS_PAR_D / 2)^k = 0.625^k still decays.
constexpr double EPS_PAR_D = 0.25;
constexpr int K_LONG_D = 18;

/// _pshift: coefficients (ascending) of P(tau + c), P of length n.
void pshift(const double* P, const int n, const double c, double* out)
{
    double cp[JCAP];
    for (int m = 0; m < n; ++m) cp[m] = std::pow(c, double(m)); // c ** (j - i)
    for (int i = 0; i < n; ++i) out[i] = 0.;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i <= j; ++i) out[i] += P[j] * binom(j, i) * cp[j - i];
    }
}

/// _pmul: the product polynomial; returns its length.
int pmul(const double* a, const int na, const double* b, const int nb, double* c)
{
    const int n = na + nb - 1;
    for (int k = 0; k < n; ++k) {
        double s = 0.;
        for (int j = std::max(0, k - nb + 1); j <= std::min(k, na - 1); ++j) s += a[j] * b[k - j];
        c[k] = s;
    }
    return n;
}

/// _M_closed: [int u^i (u^2 + k^2)^(power/2) du for i = 0..imax] (indefinite).
void M_closed(const double u, const double k, const int imax, const int power, double* M)
{
    const double S = std::hypot(u, k);
    if (power == 1) {
        const double S3 = std::pow(S, 3.);
        M[0] = 0.5 * (u * S + (k > 0 ? k * k * std::asinh(u / k) : 0.));
        if (imax >= 1) M[1] = S3 / 3.;
        for (int i = 2; i <= imax; ++i) {
            M[i] = (std::pow(u, double(i - 1)) * S3 - (i - 1) * k * k * M[i - 2]) / (i + 2);
        }
    } else {
        M[0] = std::asinh(u / k);
        if (imax >= 1) M[1] = S;
        for (int i = 2; i <= imax; ++i) {
            M[i] = (std::pow(u, double(i - 1)) * S - (i - 1) * k * k * M[i - 2]) / i;
        }
    }
}

/// seg_int: int_0^ell P(sigma) S(sigma)^power d sigma, S(sigma) = |w0 + sigma tp|, power = +1 or
/// -1, P ascending in sigma (length np). With power = -1 the caller guarantees P/S is bounded (P
/// vanishes where S does). Short against the distance to the complex zeros of S^2 (ell |tp| <=
/// SHORT S(mid)): Taylor series of the integrand about the midpoint, ratio <= SHORT/2, K_SEG + np
/// terms. Otherwise closed forms in u (S = sqrt(u^2 + k^2)), binomial sums bounded by
/// (1 + 2/SHORT)^deg.
double seg_int(
    const Vec3& w0,
    const Vec3& tp,
    const double ell,
    const double* P,
    const int np,
    const int power)
{
    const Vec3 wm = w0 + 0.5 * ell * tp;
    const double Sm = wm.norm();
    const double sa = tp.norm();
    double Pc[JCAP];
    pshift(P, np, 0.5 * ell, Pc); // in tau = sigma - ell/2
    const auto even_sum = [&]() {
        double s = 0.;
        for (int j = 0; j < np; j += 2) {
            s += Pc[j] * 2 * std::pow(0.5 * ell, double(j + 1)) / double(j + 1);
        }
        return s;
    };
    if (sa <= 1e-14 && Sm <= 1e-14 * ell) {
        // the segment lies on the zero set of S up to rounding (an edge on the input edge line or
        // through the vertex): S <= 2e-14 ell on it; for power -1 the caller's P vanishes as well
        return power == -1 ? 0. : Sm * even_sum();
    }
    if (sa == 0.) return (power == 1 ? Sm : 1. / Sm) * even_sum(); // S constant
    if (Sm > 0 && sa * ell <= SHORT * Sm) {
        // Taylor series in tau' = tau / (ell/2) in [-1, 1] (scaled, so no coefficient overflows)
        const int K = K_SEG + np;
        const double hl = 0.5 * ell;
        Jet S2{}, S, Ps{}, g;
        S2[0] = Sm * Sm;
        S2[1] = 2 * wm.dot(tp) * hl;
        S2[2] = (sa * hl) * (sa * hl);
        jsqrt(S2.data(), K, S.data());
        if (power == -1) {
            Jet e0{}, Si;
            e0[0] = 1.;
            jdiv(e0.data(), S.data(), K, Si.data());
            S = Si;
        }
        for (int j = 0; j < np; ++j) Ps[size_t(j)] = Pc[j] * std::pow(hl, double(j));
        for (int k = 0; k < K; ++k) { // jmul(Ps, S), Ps vanishing beyond its first np: zeros skipped
            double sk = 0.;
            for (int j = 0; j <= std::min(k, np - 1); ++j) sk += Ps[size_t(j)] * S[size_t(k - j)];
            g[size_t(k)] = sk;
        }
        double sum = 0.;
        for (int i = 0; i < K; i += 2) sum += g[size_t(i)] * 2. / double(i + 1);
        return hl * sum;
    }
    const double um = wm.dot(tp) / sa;
    const double k = wm.cross(tp).norm() / sa;
    const double u1 = um - 0.5 * sa * ell, u2 = um + 0.5 * sa * ell;
    const int deg = np - 1;
    if (power == -1 && k <= 16 * EPS * (std::abs(um) + sa * ell)) {
        // the segment meets the zero of S at u = 0: P/S = Q(tau) sign(u) / sa with P = (tau - tau0) Q
        const double tau0 = -um / sa;
        double out[JCAP];
        double acc = 0.;
        for (int j = deg; j >= 0; --j) { // synthetic division by (tau - tau0), descending
            acc = acc * tau0 + Pc[j];
            out[deg - j] = acc;
        }
        double Q[JCAP]; // quotient, ascending: out[:-1] reversed
        for (int j = 0; j < deg; ++j) Q[j] = out[deg - 1 - j];
        const auto F = [&](const double t) {
            double s = 0.;
            for (int j = 0; j < deg; ++j) s += Q[j] * std::pow(t, double(j + 1)) / double(j + 1);
            return s;
        };
        const double lo = -0.5 * ell, hi = 0.5 * ell;
        const double t0 = std::min(std::max(tau0, lo), hi);
        return ((F(hi) - F(t0)) - (F(t0) - F(lo))) / sa;
    }
    double M1[JCAP], M2[JCAP];
    M_closed(u1, k, deg, power, M1);
    M_closed(u2, k, deg, power, M2);
    double tot = 0.;
    for (int j = 0; j <= deg; ++j) {
        if (Pc[j] == 0.) continue;
        double sj = 0.;
        for (int i = 0; i <= j; ++i) {
            sj += binom(j, i) * std::pow(-um, double(j - i)) * (M2[i] - M1[i]);
        }
        tot += Pc[j] / std::pow(sa, double(j + 1)) * sj;
    }
    return tot;
}

/// face_inv_point: int_F 1/|x - v| dA: radial field (R - |z|)/rho^2 (y - q), divergence 1/R; per
/// edge h [asinh(s2/a) - asinh(s1/a)] + |z| [atan(s c/(h R)) - atan(s/h)] (rationalized).
double face_inv_point(const Vec3* V, const int k, const Vec3& n, const Vec3& v)
{
    const double z0 = n.dot(V[0] - v);
    const double c = std::abs(z0);
    const Vec3 q = v + z0 * n;
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m);
        if (h == 0.) continue;
        const double s1 = (x1 - q).dot(t);
        const double s2 = s1 + ell;
        const double a = std::hypot(h, c);
        const double R1 = std::hypot(s1, a), R2 = std::hypot(s2, a);
        double num, ash;
        if (s1 * s2 > 0.) {
            const double den = s2 * R1 + s1 * R2;
            num = a * a * ell * (s1 + s2) / den;
            ash = std::asinh(ell * (s1 + s2) / den);
        } else {
            num = s2 * R1 - s1 * R2;
            ash = std::asinh(s2 / a) - std::asinh(s1 / a);
        }
        double term = h * ash;
        if (c > 0.) {
            term += c * (std::atan2(c * num * h, h * h * R1 * R2 + s1 * s2 * c * c) -
                         std::atan2(ell * h, h * h + s1 * s2));
        }
        tot += term;
    }
    return tot;
}

/// face_point_derivs: (P1, P2) = (int_F lam R dA, int_F lam^2 grad R dA), R = |y - v|,
/// lam = l0 + g0.y. Uses grad_Pi R = (y - q)/R, (y - q) R = grad_Pi R^3/3 and
/// a.grad[(a.(y - c)) R] = |a|^2 R + (a.(y - c))(a.(y - q))/R, expanded about the face centre c
/// (expanding about the foot q instead loses (|q - F|/diam F)^2).
void face_point_derivs(
    const Vec3* V,
    const int k,
    const Vec3& n,
    const Vec3& v,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2,
    const Ctx& cx)
{
    const double z0 = n.dot(V[0] - v);
    const Vec3 q = v + z0 * n;
    Vec3 c = Vec3::Zero();
    for (int i = 0; i < k; ++i) c += V[i];
    c /= double(k);
    const Vec3 a = g0 - g0.dot(n) * n; // in-plane gradient of lam
    const double lc = l0 + g0.dot(c);
    const double IR = face_point(V, k, n, v, cx);
    Vec3 E1 = Vec3::Zero(), E3 = Vec3::Zero(), El = Vec3::Zero();
    double Ea = 0.;
    const double one[1] = {1.};
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const Vec3 w0 = x1 - v;
        const double R2[3] = {w0.dot(w0), 2 * w0.dot(t), 1.}; // |y - v|^2 along the edge
        const double lam[2] = {l0 + g0.dot(x1), g0.dot(t)};
        const double La[2] = {a.dot(x1 - c), a.dot(t)};
        double lam2[3];
        pmul(lam, 2, lam, 2, lam2);
        E1 += m * seg_int(w0, t, ell, one, 1, 1);
        E3 += m * seg_int(w0, t, ell, R2, 3, 1);
        Ea += a.dot(m) * seg_int(w0, t, ell, La, 2, 1);
        El += m * seg_int(w0, t, ell, lam2, 3, 1);
    }
    const double aqc = a.dot(q - c);
    P1 = lc * IR + a.dot(E3) / 3. + aqc * IR;
    double lam2_over_R = 0.;
    if (z0 != 0.) {
        const double I1 = face_inv_point(V, k, n, v);
        const double B = a.dot(E1) + aqc * I1; // int a.(y - c) / R
        const double C = (Ea - a.dot(a) * IR) + aqc * B; // int (a.(y - c))^2 / R
        lam2_over_R = lc * lc * I1 + 2 * lc * B + C;
    }
    P2 = El - 2 * a * P1 + z0 * n * lam2_over_R;
}

/// _jXi: jets of Xi_m, m = 0..mmax, antiderivatives in eta at fixed zeta > 0:
/// kind 0 ('r'):  Xi_m = int eta^m r d eta,         Xi_0 = int_0^eta r = G;
/// kind 1 ('zr'): Xi_m = zeta int eta^m / r d eta,  Xi_0 = zeta asinh(eta/zeta);
/// r = sqrt(eta^2 + zeta^2). Recursions from d(eta^(m-1) r^3) and d(eta^(m-1) r). Xi has room for
/// max(mmax + 1, 2) jets of length n. eta and zeta are linear (every caller's: e0 + e1 l), so the
/// products with them, with zeta^2 and the quotient by zeta skip their zero coefficients (jmul3,
/// jdiv2): the same sums in the same order, the same values (MEASURED: bit-identical on every
/// stored band cell).
constexpr int MAXXI = 2 + K_LONG_D + 2 + 3; ///< mmax + 1 <= Aa + (K_LONG_D + degw) + 2 = 25
void jXi(const double* eta, const double* zeta, const int n, const int mmax, const int kind, Jet* Xi)
{
    Jet u, A, w, R, z2, t, t2;
    jdiv2(eta, zeta, n, u.data());
    jasinh_w(u.data(), n, A.data(), w.data());
    jmul3(zeta, w.data(), n, R.data());
    jmul3(zeta, zeta, n, z2.data());
    const int ne = std::max(0, mmax - 1); // ep[m] = eta^m, m = 0..ne
    std::array<Jet, MAXXI> ep;
    ep[0].fill(0.);
    ep[0][0] = 1.;
    for (int m = 1; m <= ne; ++m) jmul3(eta, ep[size_t(m - 1)].data(), n, ep[size_t(m)].data());
    if (kind == 0) {
        Jet R2, R3;
        jmul(R.data(), R.data(), n, R2.data());
        jmul(R.data(), R2.data(), n, R3.data());
        jmul(u.data(), w.data(), n, t.data());
        for (int i = 0; i < n; ++i) t[size_t(i)] += A[size_t(i)];
        jmul3(z2.data(), t.data(), n, Xi[0].data());
        for (int i = 0; i < n; ++i) Xi[0][size_t(i)] *= 0.5;
        for (int i = 0; i < n; ++i) Xi[1][size_t(i)] = R3[size_t(i)] / 3.;
        for (int m = 2; m <= mmax; ++m) {
            jmul(ep[size_t(m - 1)].data(), R3.data(), n, t.data());
            jmul3(z2.data(), Xi[m - 2].data(), n, t2.data());
            for (int i = 0; i < n; ++i) {
                Xi[m][size_t(i)] = (t[size_t(i)] - (m - 1) * t2[size_t(i)]) / (m + 2);
            }
        }
        return;
    }
    std::array<Jet, MAXXI> Y;
    Y[0] = A;
    Y[1] = R;
    for (int m = 2; m <= mmax; ++m) {
        jmul(ep[size_t(m - 1)].data(), R.data(), n, t.data());
        jmul3(z2.data(), Y[size_t(m - 2)].data(), n, t2.data());
        for (int i = 0; i < n; ++i) {
            Y[size_t(m)][size_t(i)] = (t[size_t(i)] - (m - 1) * t2[size_t(i)]) / m;
        }
    }
    for (int m = 0; m <= mmax; ++m) jmul3(zeta, Y[size_t(m)].data(), n, Xi[m].data());
}

/// The weights of _edge_np_w: W[a] ascending coefficients in xi, a = 0..na-1, each of length len[a].
struct Weights
{
    int na = 0;
    int len[3] = {0, 0, 0};
    double c[3][3] = {};
};

/// _edge_np_w: int_{xi1}^{xi2} sum_a W[a](xi) Phi_a(eta(xi), zeta(xi)) d xi along one edge, with
/// Phi_a = int_0^eta t^a Psi(t, zeta) dt, Psi = r (kind 0) or zeta / r (kind 1), zeta > 0. The same
/// two paths as edge_near_parallel(), at the fixed orders (K_SHORT + 1 + deg W, K_LONG_D + 1).
double edge_np_w(
    const double e1,
    const double e2,
    const double z1,
    const double z2,
    const double x1,
    const double x2,
    const Weights& W,
    const int kind)
{
    const double em = 0.5 * (e1 + e2), zm = 0.5 * (z1 + z2), xm = 0.5 * (x1 + x2);
    const double de = e2 - e1, dz = z2 - z1, dx = x2 - x1;
    const int Aa = W.na - 1;
    int degw = 0;
    for (int a = 0; a < W.na; ++a) degw = std::max(degw, W.len[a] - 1);
    if (std::hypot(de, dz) <= SHORT * std::hypot(em, zm)) {
        const int K = K_SHORT + 1 + degw;
        Jet ej{}, zj{}, xj{}, zero{};
        ej[0] = em;
        ej[1] = de;
        zj[0] = zm;
        zj[1] = dz;
        xj[0] = xm;
        xj[1] = dx;
        std::array<Jet, 3> Xi, Xi0;
        jXi(ej.data(), zj.data(), K, Aa, kind, Xi.data());
        jXi(zero.data(), zj.data(), K, Aa, kind, Xi0.data());
        std::array<Jet, 3> xp; // _jpow(xj, degw)
        xp[0].fill(0.);
        xp[0][0] = 1.;
        for (int m = 1; m <= degw; ++m) jmul(xp[size_t(m - 1)].data(), xj.data(), K, xp[size_t(m)].data());
        Jet g{}, wj, diff, t;
        for (int a = 0; a <= Aa; ++a) {
            wj.fill(0.);
            for (int j = 0; j < W.len[a]; ++j) {
                for (int i = 0; i < K; ++i) wj[size_t(i)] += W.c[a][j] * xp[size_t(j)][size_t(i)];
            }
            for (int i = 0; i < K; ++i) diff[size_t(i)] = Xi[size_t(a)][size_t(i)] - Xi0[size_t(a)][size_t(i)];
            jmul(wj.data(), diff.data(), K, t.data());
            for (int i = 0; i < K; ++i) g[size_t(i)] += t[size_t(i)];
        }
        double sum = 0., p = 1.;
        for (int i = 0; i < K; i += 2) {
            sum += g[size_t(i)] * p / double(i + 1);
            p *= 0.25; // 0.5^i, exact
        }
        return dx * sum;
    }
    const int K = K_LONG_D + 1;
    const double mu = dz / de, nu = dx / de;
    Jet zj{}, zero{};
    zj[0] = zm;
    zj[1] = 1.;
    const int Lmax = (K - 1) + degw;
    const int mmax = Aa + Lmax + 1;
    std::array<std::array<Jet, MAXXI>, 2> XiE;
    for (int s = 0; s < 2; ++s) {
        Jet eta{};
        eta[0] = s == 0 ? e1 : e2;
        jXi(eta.data(), zj.data(), K + 1, mmax, kind, XiE[size_t(s)].data());
    }
    std::array<Jet, 3> Xi0;
    jXi(zero.data(), zj.data(), K + 1, Aa, kind, Xi0.data());
    double tot = 0.;
    double Q[JCAP], c[JCAP];
    for (int a = 0; a <= Aa; ++a) {
        double wt[3];
        pshift(W.c[a], W.len[a], xm, wt); // W[a](xm + nu tau)
        for (int j = 0; j < W.len[a]; ++j) wt[j] *= std::pow(nu, double(j));
        for (int k = 0; k < K; ++k) {
            const int nq = k + W.len[a]; // tau^k W(...)
            for (int i = 0; i < k; ++i) Q[i] = 0.;
            for (int j = 0; j < W.len[a]; ++j) Q[k + j] = wt[j];
            pshift(Q, nq, -em, c); // in powers of eta
            double sk = 0.;
            for (int l = 0; l < nq; ++l) {
                const double cl = c[l];
                if (cl == 0.) continue;
                const double p1 = std::pow(e1, double(l + 1)), p2 = std::pow(e2, double(l + 1));
                const double F0 = (p1 * XiE[0][size_t(a)][size_t(k)] - XiE[0][size_t(a + l + 1)][size_t(k)]) / (l + 1);
                const double F1 = (p2 * XiE[1][size_t(a)][size_t(k)] - XiE[1][size_t(a + l + 1)][size_t(k)]) / (l + 1);
                sk += cl * (F1 - F0 - Xi0[size_t(a)][size_t(k)] * (p2 - p1) / (l + 1));
            }
            tot += std::pow(mu, double(k)) * sk;
        }
    }
    return dx / de * tot;
}

/// _poly_polygon: int over the polygon V of a quadratic polynomial f, exact: fan triangles,
/// edge-midpoint rule (exact for degree <= 2).
template <typename F>
double poly_polygon(const Vec3* V, const int k, F&& f)
{
    double tot = 0.;
    for (int i = 1; i < k - 1; ++i) {
        const Vec3 &a = V[0], &b = V[i], &c = V[i + 1];
        const double area = 0.5 * (b - a).cross(c - a).norm();
        tot = tot + area * ((f(Vec3((a + b) / 2)) + f(Vec3((b + c) / 2)) + f(Vec3((c + a) / 2))) / 3.);
    }
    return tot;
}

/// _split_polygon: the polygon V cap {g.x <= h} (2D clip in its plane); false if fewer than 3
/// vertices remain.
bool split_polygon(const Vec3* V, const int k, const Vec3& g, const double h, std::vector<Vec3>& out)
{
    out.clear();
    thread_local std::vector<double> sv;
    sv.resize(size_t(k));
    double* sp = sv.data();
    for (int i = 0; i < k; ++i) sp[i] = V[i].dot(g) - h;
    for (int i = 0; i < k; ++i) {
        const int j = (i + 1) % k;
        if (sp[i] <= 0) out.push_back(V[i]);
        if ((sp[i] < 0 && 0 < sp[j]) || (sp[j] < 0 && 0 < sp[i])) {
            out.push_back(V[i] + (sp[i] / (sp[i] - sp[j])) * (V[j] - V[i]));
        }
    }
    return out.size() >= 3;
}

/// face_line_derivs: (P1, P2) = (int_F lam r dA, int_F lam^2 grad r dA), r = distance to the line
/// {A + l e}; the same three branches as face_line(), the series up to EPS_PAR_D.
void face_line_derivs(
    const Vec3* V,
    const int k,
    const Vec3& n,
    const Vec3& A,
    const Vec3& e,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2)
{
    const double sn = e.dot(n);
    const Vec3 a = g0 - g0.dot(n) * n;
    const auto lamf = [&](const Vec3& Y) { return l0 + Y.dot(g0); };
    double vmax = 0.;
    for (int i = 0; i < k; ++i) vmax = std::max(vmax, maxabs(V[i]));
    const double scale = vmax + maxabs(A);
    Vec3 inplane_edges = Vec3::Zero();
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        Vec3 w0 = x1 - A;
        w0 = w0 - w0.dot(e) * e;
        const Vec3 tp = t - t.dot(e) * e;
        const double lam[2] = {l0 + g0.dot(x1), g0.dot(t)};
        double lam2[3];
        pmul(lam, 2, lam, 2, lam2);
        inplane_edges += t.cross(n) * seg_int(w0, tp, ell, lam2, 3, 1);
    }
    if (std::abs(sn) <= 16 * EPS && std::abs(n.dot(V[0] - A)) <= 16 * EPS * scale) {
        // the face plane contains the line: r = |eta|, grad r = sign(eta) e_eta in the plane
        Vec3 eh = n.cross(e);
        eh /= eh.norm();
        P1 = 0.;
        P2.setZero();
        std::vector<Vec3> Vs;
        for (const double sg : {1., -1.}) {
            if (split_polygon(V, k, -sg * eh, -sg * eh.dot(A), Vs)) {
                P1 += poly_polygon(Vs.data(), int(Vs.size()), [&](const Vec3& Y) {
                    return lamf(Y) * sg * ((Y - A).dot(eh));
                });
                P2 = P2 + sg * eh * poly_polygon(Vs.data(), int(Vs.size()), [&](const Vec3& Y) {
                         const double l = lamf(Y);
                         return l * l;
                     });
            }
        }
        return;
    }
    if (std::abs(sn) < 0.5) {
        const Vec3 ep = e - sn * n;
        const double cth = ep.norm();
        const Vec3 exi = ep / cth;
        const Vec3 eeta = n.cross(exi);
        Vec3 vm = Vec3::Zero();
        for (int i = 0; i < k; ++i) vm += V[i];
        vm /= double(k);
        const Vec3 Ac = A + (vm - A).dot(e) * e;
        const double H = n.dot(V[0] - Ac);
        const auto xi_of = [&](int i) { return (V[i] - Ac).dot(exi); };
        const auto eta_of = [&](int i) { return (V[i] - Ac).dot(eeta); };
        const auto zeta_of = [&](double xi) { return xi * sn - H * cth; };
        double ximin = std::numeric_limits<double>::infinity(), ximax = -ximin;
        double zmin = ximin, zmax = -ximin;
        for (int i = 0; i < k; ++i) {
            const double xi = xi_of(i), z = zeta_of(xi);
            ximin = std::min(ximin, xi);
            ximax = std::max(ximax, xi);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        if (zmin > 0. || zmax < 0.) {
            const double sg = zeta_of(xi_of(0)) > 0 ? 1. : -1.;
            const double zamin = sg > 0 ? zmin : -zmax;
            if (std::abs(sn) * (ximax - ximin) < EPS_PAR_D * zamin) {
                const double lA = l0 + g0.dot(Ac) + H * g0.dot(n);
                const double le = g0.dot(eeta), lx = g0.dot(exi);
                Weights W1, W2;
                W1.na = 2;
                W1.len[0] = 2;
                W1.c[0][0] = lA;
                W1.c[0][1] = lx;
                W1.len[1] = 1;
                W1.c[1][0] = le;
                W2.na = 3;
                W2.len[0] = 3;
                W2.c[0][0] = lA * lA;
                W2.c[0][1] = 2 * lA * lx;
                W2.c[0][2] = lx * lx;
                W2.len[1] = 2;
                W2.c[1][0] = 2 * le * lA;
                W2.c[1][1] = 2 * le * lx;
                W2.len[2] = 1;
                W2.c[2][0] = le * le;
                P1 = 0.;
                double Z = 0.;
                for (int i = 0; i < k; ++i) {
                    const int j = (i + 1) % k;
                    const double xii = xi_of(i), xij = xi_of(j);
                    if (xij != xii) {
                        const double ei = eta_of(i), ej = eta_of(j);
                        const double zi = sg * zeta_of(xii), zj = sg * zeta_of(xij);
                        P1 -= edge_np_w(ei, ej, zi, zj, xii, xij, W1, 0);
                        Z -= edge_np_w(ei, ej, zi, zj, xii, xij, W2, 1);
                    }
                }
                // n.w = -cos(theta) zeta on the plane; Z = int lam^2 za / r, zeta = sg za
                P2 = inplane_edges - 2 * a * P1 + n * (-cth * sg * Z);
                return;
            }
        }
    }
    // radial branch: r is homogeneous of degree 1 about q in the face plane
    const Vec3 q = A + (n.dot(V[0] - A) / sn) * e;
    const double lq = l0 + g0.dot(q);
    P1 = 0.;
    double Pn = 0.;
    const double one[1] = {1.};
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m);
        if (h == 0.) continue;
        Vec3 w0 = x1 - A;
        w0 = w0 - w0.dot(e) * e;
        const Vec3 tp = t - t.dot(e) * e;
        const double L1[2] = {a.dot(x1 - q), a.dot(t)}; // a.(y - q) along the edge
        const double nw[2] = {-sn * e.dot(x1 - q), -sn * e.dot(t)}; // n.w along the edge
        double L1nw[3], L1L1[3], L1L1nw[4];
        pmul(L1, 2, nw, 2, L1nw);
        pmul(L1, 2, L1, 2, L1L1);
        pmul(L1L1, 3, nw, 2, L1L1nw);
        P1 += h * (lq * seg_int(w0, tp, ell, one, 1, 1) / 3. + seg_int(w0, tp, ell, L1, 2, 1) / 4.);
        Pn += h * (lq * lq * seg_int(w0, tp, ell, nw, 2, -1) / 2. +
                   2 * lq * seg_int(w0, tp, ell, L1nw, 3, -1) / 3. +
                   seg_int(w0, tp, ell, L1L1nw, 4, -1) / 4.);
    }
    P2 = inplane_edges - 2 * a * P1 + n * Pn;
}

/// An elementary piece (exact_mesh.elementary): a convex piece on which the integrand is ONE
/// smooth function phi: kind 0 affine phi = W.y - c (W in `dir`), 1 the distance to the line
/// {p + l dir}, 2 the distance to the point p. `rule_a`: the member of smallest integral on a
/// rule-(a) piece (a bound).
struct Elem
{
    Poly Q;
    int kind = 0;
    Vec3 p, dir;
    double c = 0.;
    bool rule_a = false;
};

/// _piece_value
double piece_value(const Elem& e, const Ctx& cx)
{
    if (e.kind == 0) {
        const auto [vol, cen] = volume_centroid(e.Q);
        return vol * (e.dir.dot(cen) - e.c);
    }
    if (e.kind == 1) return vol_line(e.Q, e.p, e.dir, cx);
    return vol_point(e.Q, e.p, cx);
}

/// _moments: (P1, P2) on the trace polygon Vf of a piece: int lam phi, int lam^2 grad phi.
void moments(
    const Vec3* Vf,
    const int k,
    const Vec3& nf,
    const Elem& e,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2,
    const Ctx& cx)
{
    const auto lam = [&](const Vec3& Y) { return l0 + Y.dot(g0); };
    if (e.kind == 0) {
        P1 = poly_polygon(Vf, k, [&](const Vec3& Y) { return lam(Y) * (Y.dot(e.dir) - e.c); });
        P2 = e.dir * poly_polygon(Vf, k, [&](const Vec3& Y) {
                 const double l = lam(Y);
                 return l * l;
             });
        return;
    }
    if (e.kind == 1) {
        face_line_derivs(Vf, k, nf, e.p, e.dir, l0, g0, P1, P2);
        return;
    }
    face_point_derivs(Vf, k, nf, e.p, l0, g0, P1, P2, cx);
}

/// _cut_integral: int over the segment [y1, y2] of phi lam^2.
double cut_integral(const Vec3& y1, const Vec3& y2, const Elem& e, const double l0, const Vec3& g0)
{
    if (e.kind == 0) {
        // _seg_poly3: Simpson, exact for degree <= 3
        const auto f = [&](const Vec3& y) {
            const double l = l0 + g0.dot(y);
            return (e.dir.dot(y) - e.c) * (l * l);
        };
        const double ell = (y2 - y1).norm();
        return ell * (f(y1) + 4 * f(Vec3(0.5 * (y1 + y2))) + f(y2)) / 6.;
    }
    const Vec3 d = y2 - y1;
    const double ell = d.norm();
    const Vec3 t = d / ell;
    const double lam[2] = {l0 + g0.dot(y1), g0.dot(t)};
    double lam2[3];
    pmul(lam, 2, lam, 2, lam2);
    if (e.kind == 1) {
        const Vec3 w0 = y1 - e.p;
        return seg_int(w0 - w0.dot(e.dir) * e.dir, t - t.dot(e.dir) * e.dir, ell, lam2, 3, 1);
    }
    return seg_int(y1 - e.p, t, ell, lam2, 3, 1);
}


} // namespace

// ------------------------------------------------------------------------------------------------
// ExactBandIntegral
// ------------------------------------------------------------------------------------------------

ExactBandIntegral::ExactBandIntegral(const Eigen::MatrixXd& Vin, const Eigen::MatrixXi& Fin)
{
    // Mesh.__init__: np.unique(V[F], axis=0) -- the referenced vertices, merged by exact
    // coordinates and sorted lexicographically.
    const int nf = int(Fin.rows());
    std::vector<Vec3> corner(size_t(3 * nf));
    for (int f = 0; f < nf; ++f) {
        for (int c = 0; c < 3; ++c) corner[size_t(3 * f + c)] = Vin.row(Fin(f, c)).head<3>();
    }
    std::vector<int> ord(corner.size());
    std::iota(ord.begin(), ord.end(), 0);
    const auto lex = [&](int a, int b) {
        const Vec3 &p = corner[size_t(a)], &q = corner[size_t(b)];
        if (p[0] != q[0]) return p[0] < q[0];
        if (p[1] != q[1]) return p[1] < q[1];
        return p[2] < q[2];
    };
    std::sort(ord.begin(), ord.end(), lex);
    std::vector<Vec3> V;
    std::vector<int> inv(corner.size());
    for (size_t i = 0; i < ord.size(); ++i) {
        if (i == 0 || lex(ord[i - 1], ord[i])) V.push_back(corner[size_t(ord[i])]);
        inv[size_t(ord[i])] = int(V.size()) - 1;
    }
    std::vector<std::array<int, 3>> F(static_cast<size_t>(nf));
    for (int f = 0; f < nf; ++f) {
        for (int c = 0; c < 3; ++c) F[size_t(f)][size_t(c)] = inv[size_t(3 * f + c)];
    }
    const auto unit = [](const Vec3& v) -> Vec3 { return v / v.norm(); };

    // edges in first-insertion order (the Python dict), with their incident (face, third vertex)
    std::map<std::pair<int, int>, int> eid;
    std::vector<std::pair<int, int>> E;
    std::vector<std::vector<std::pair<int, int>>> E_faces;
    m_face_feats.resize(size_t(nf));
    for (int f = 0; f < nf; ++f) {
        for (int k = 0; k < 3; ++k) {
            const int a = F[size_t(f)][size_t(k)], b = F[size_t(f)][size_t((k + 1) % 3)],
                      c = F[size_t(f)][size_t((k + 2) % 3)];
            const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
            auto it = eid.find(key);
            if (it == eid.end()) {
                it = eid.emplace(key, int(E.size())).first;
                E.push_back(key);
                E_faces.emplace_back();
            }
            E_faces[size_t(it->second)].push_back({f, c});
            m_face_feats[size_t(f)][size_t(k)] = it->second;
            m_face_feats[size_t(f)][size_t(3 + k)] = a;
        }
    }
    std::vector<std::vector<int>> vedges(V.size());
    for (const auto& [a, b] : E) {
        vedges[size_t(a)].push_back(b);
        vedges[size_t(b)].push_back(a);
    }
    m_nF = nf;
    m_nE = int(E.size());
    for (auto& ff : m_face_feats) {
        for (int k = 0; k < 3; ++k) ff[size_t(k)] += m_nF;
        for (int k = 3; k < 6; ++k) ff[size_t(k)] += m_nF + m_nE;
    }

    // features: (kind, data, half-spaces); region = {x : g.(x - q) <= 0 for all (g, q)}
    for (int f = 0; f < nf; ++f) {
        const Vec3 a[3] = {
            V[size_t(F[size_t(f)][0])],
            V[size_t(F[size_t(f)][1])],
            V[size_t(F[size_t(f)][2])]};
        Feature ft;
        ft.kind = 0;
        ft.dir = unit((a[1] - a[0]).cross(a[2] - a[0]));
        ft.p = a[0];
        ft.h_begin = int(m_hs.size());
        for (int k = 0; k < 3; ++k) {
            const Vec3 &p = a[k], &q = a[(k + 1) % 3], &r = a[(k + 2) % 3];
            Vec3 m = unit((q - p).cross(ft.dir));
            if (m.dot(r - p) > 0) m = -m;
            m_hs.push_back({m, p});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = a[0].cwiseMin(a[1]).cwiseMin(a[2]);
        ft.hi = a[0].cwiseMax(a[1]).cwiseMax(a[2]);
        m_feat.push_back(ft);
        m_A.push_back(a[0]);
        m_B.push_back(a[1]);
        m_C.push_back(a[2]);
    }
    for (size_t i = 0; i < E.size(); ++i) {
        const Vec3 &a = V[size_t(E[i].first)], &b = V[size_t(E[i].second)];
        Feature ft;
        ft.kind = 1;
        ft.p = a;
        ft.dir = unit(b - a);
        const Vec3& e = ft.dir;
        ft.h_begin = int(m_hs.size());
        m_hs.push_back({-e, a});
        m_hs.push_back({e, b});
        for (const auto& [fi, ic] : E_faces[i]) {
            const Vec3 w = a - V[size_t(ic)];
            const Vec3 m = unit(w - w.dot(e) * e); // in the face plane, away from c
            m_hs.push_back({-m, a});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = a.cwiseMin(b);
        ft.hi = a.cwiseMax(b);
        m_feat.push_back(ft);
    }
    for (size_t iv = 0; iv < V.size(); ++iv) {
        const Vec3& v = V[iv];
        Feature ft;
        ft.kind = 2;
        ft.p = v;
        ft.dir.setZero();
        ft.h_begin = int(m_hs.size());
        for (const int w : vedges[iv]) {
            const Vec3 g = unit(V[size_t(w)] - v);
            m_hs.push_back({g, v});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = ft.hi = v;
        m_feat.push_back(ft);
    }

    // the triangle tree (InputTriangles' construction)
    if (nf == 0) return;
    std::vector<Vec3> centroid(static_cast<size_t>(nf));
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity()), hi = -lo;
    for (int f = 0; f < nf; ++f) {
        centroid[size_t(f)] = (m_A[size_t(f)] + m_B[size_t(f)] + m_C[size_t(f)]) / 3.;
        lo = lo.cwiseMin(m_feat[size_t(f)].lo);
        hi = hi.cwiseMax(m_feat[size_t(f)].hi);
    }
    m_order.resize(size_t(nf));
    std::iota(m_order.begin(), m_order.end(), 0);
    m_nodes.reserve(size_t(2 * nf));
    build_node(0, nf, centroid, 1e-9 * std::max((hi - lo).norm(), 1e-300));
}

int ExactBandIntegral::build_node(
    const int begin,
    const int end,
    const std::vector<Eigen::Vector3d>& centroid,
    const double pad)
{
    const int id = int(m_nodes.size());
    m_nodes.emplace_back();
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 hi = -lo, clo = lo, chi = -lo;
    for (int j = begin; j < end; ++j) {
        const int f = m_order[size_t(j)];
        lo = lo.cwiseMin(m_feat[size_t(f)].lo);
        hi = hi.cwiseMax(m_feat[size_t(f)].hi);
        clo = clo.cwiseMin(centroid[size_t(f)]);
        chi = chi.cwiseMax(centroid[size_t(f)]);
    }
    m_nodes[size_t(id)].lo = lo - Vec3::Constant(pad);
    m_nodes[size_t(id)].hi = hi + Vec3::Constant(pad);
    m_nodes[size_t(id)].begin = begin;
    m_nodes[size_t(id)].end = end;
    if (end - begin <= 4) return id;
    int axis = 0;
    (chi - clo).maxCoeff(&axis);
    const int mid = (begin + end) / 2;
    std::nth_element(
        m_order.begin() + begin,
        m_order.begin() + mid,
        m_order.begin() + end,
        [&](const int a, const int b) {
            return centroid[size_t(a)][axis] < centroid[size_t(b)][axis] ||
                   (centroid[size_t(a)][axis] == centroid[size_t(b)][axis] && a < b);
        });
    const int left = build_node(begin, mid, centroid, pad);
    const int right = build_node(mid, end, centroid, pad);
    m_nodes[size_t(id)].left = left;
    m_nodes[size_t(id)].right = right;
    return id;
}

double ExactBandIntegral::upper_bound(const std::array<Eigen::Vector3d, 4>& t) const
{
    // Branch and bound: max over the corners of the distance to a node's (padded) box is a lower
    // bound of max_i d_P(t_i) for every triangle P in it, so the result is the exact minimum.
    double m = std::numeric_limits<double>::infinity();
    if (m_nodes.empty()) return m;
    const auto lower = [&](const Node& nd) {
        double s = 0.;
        for (const Vec3& p : t) s = std::max(s, (nd.lo - p).cwiseMax(p - nd.hi).cwiseMax(0.).norm());
        return s;
    };
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > m) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) {
                const int f = m_order[size_t(j)];
                double s = 0.;
                for (const Vec3& p : t) {
                    s = std::max(s, point_tri_dist(p, m_A[size_t(f)], m_B[size_t(f)], m_C[size_t(f)]));
                    if (s >= m) break; // this triangle cannot lower the min: the result is unchanged
                }
                m = std::min(m, s);
            }
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    return m;
}

void ExactBandIntegral::candidates(
    const Eigen::Vector3d& tlo,
    const Eigen::Vector3d& thi,
    const double r,
    std::vector<int>& out) const
{
    // Faces from the tree; then their edges and vertices. A face's box contains its edges' and
    // vertices' boxes, so its box distance is never larger: every edge or vertex within r has an
    // incident face within r (exactly so in floating point: the per-axis gaps are monotone).
    out.clear();
    if (m_nodes.empty()) return;
    std::vector<int> stack = {0};
    while (!stack.empty()) {
        const Node& nd = m_nodes[size_t(stack.back())];
        stack.pop_back();
        if (box_dist(nd.lo, nd.hi, tlo, thi) > r) continue;
        if (nd.left >= 0) {
            stack.push_back(nd.left);
            stack.push_back(nd.right);
            continue;
        }
        for (int j = nd.begin; j < nd.end; ++j) {
            const int f = m_order[size_t(j)];
            if (box_dist(m_feat[size_t(f)].lo, m_feat[size_t(f)].hi, tlo, thi) > r) continue;
            out.push_back(f);
            for (const int g : m_face_feats[size_t(f)]) {
                if (box_dist(m_feat[size_t(g)].lo, m_feat[size_t(g)].hi, tlo, thi) <= r) {
                    out.push_back(g);
                }
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

struct ExactBandIntegral::Work
{
    Vec3 c0; ///< the local origin: t's centroid
    Poly Q0; ///< t in local coordinates
    std::vector<Loc> loc; ///< the candidate features, local
    std::vector<std::pair<Vec3, double>> hs; ///< their half-spaces, local
    std::vector<Piece> pieces; ///< the partition of t with member sets
    double tol_cut = 0., tol_any = 0.; ///< the partition's tolerance budget (Ctx)
    double scale = 0.; ///< the cell's rounding scale (Ctx::scale)
};

bool ExactBandIntegral::partition(const std::array<Eigen::Vector3d, 4>& T, Work& w, Stats* stats)
    const
{
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    w.tol_cut = w.tol_any = 0.;
    cx.tol_cut = &w.tol_cut;
    cx.tol_any = &w.tol_any;
    // _partition, step 0: candidates in local coordinates (origin c0 = centroid of t). A flat
    // cell is 0 before any search (the Python searches first; the value is the same).
    w.c0 = (((T[0] + T[1]) + T[2]) + T[3]) / 4.;
    const Vec3& c0 = w.c0;
    std::array<Vec3, 4> Tl;
    for (int i = 0; i < 4; ++i) Tl[size_t(i)] = T[size_t(i)] - c0;
    if (!tet_polytope(Tl, w.Q0)) return false;
    std::vector<Loc>& loc = w.loc;
    std::vector<std::pair<Vec3, double>>& hs = w.hs;
    loc.clear();
    hs.clear();
    w.scale = 0.;
    {
        Timer tm(cx, &Stats::t_candidates);
        Vec3 tlo = T[0], thi = T[0];
        for (const Vec3& p : T) {
            tlo = tlo.cwiseMin(p);
            thi = thi.cwiseMax(p);
        }
        const double U = upper_bound(T);
        std::vector<int> cand;
        candidates(tlo, thi, U * (1 + 1e-12), cand);
        loc.reserve(cand.size());
        for (const int gi : cand) {
            const Feature& ft = m_feat[size_t(gi)];
            Loc l;
            l.kind = ft.kind;
            l.p = ft.p - c0;
            l.dir = ft.dir;
            l.h_begin = int(hs.size());
            for (int i = ft.h_begin; i < ft.h_end; ++i) {
                const auto& [g, q] = m_hs[size_t(i)];
                hs.push_back({g, g.dot(q - c0)}); // in the cell's frame: see m_hs
                w.scale = std::max(w.scale, maxabs(q - c0));
            }
            l.h_end = int(hs.size());
            loc.push_back(l);
        }
        if (stats) stats->candidates += int64_t(cand.size());
    }
    // The cell's rounding scale: the largest magnitude among the terms its planes and vertices are
    // built from, max |t_i - c0| (the corners) and max |q - c0| (the points the candidates'
    // half-space offsets h = g.(q - c0) are taken from), max norm (exact_mesh._cell_scale). A
    // tolerance on the clipped polytope's own coordinates alone (|h| + max |V|) missed what the
    // offsets and the inherited vertices carry. MEASURED: on an inward cylinder cell the
    // candidates' points lay 0.355 from c0 while a clip saw 3.9e-3; two copies of one vertex,
    // made by different clips 3-6 tol apart, fell on both sides of a plane parallel to a face
    // 1.9 tol away, the faces disagreed about the cut, and the pieces lost 3.8e-4 of the cell.
    for (const Vec3& p : Tl) w.scale = std::max(w.scale, maxabs(p));
    cx.scale = w.scale;

    // step 1: the convex pieces of t with constant member sets
    std::vector<Piece>& pieces = w.pieces;
    std::vector<Piece> next;
    pieces.assign(1, Piece());
    pieces[0].Q = w.Q0;
    std::vector<Poly> outs;
    for (int k = 0; k < int(loc.size()); ++k) {
        const std::pair<Vec3, double>* H = hs.data() + loc[size_t(k)].h_begin;
        const int nh = loc[size_t(k)].h_end - loc[size_t(k)].h_begin;
        next.clear();
        for (Piece& pc : pieces) {
            const Side st = classify(pc.Q, H, nh, cx.tol_any, cx.scale);
            if (st == Side::In) {
                pc.S.push_back(k);
                next.push_back(std::move(pc));
            } else if (st == Side::Out) {
                next.push_back(std::move(pc));
            } else {
                Piece in;
                outs.clear();
                split(pc.Q, H, nh, in.Q, outs, cx);
                if (in.Q.faces() > 0) {
                    in.S = pc.S;
                    in.S.push_back(k);
                    next.push_back(std::move(in));
                }
                for (Poly& qo : outs) {
                    Piece o;
                    o.Q = std::move(qo);
                    o.S = pc.S;
                    next.push_back(std::move(o));
                }
            }
        }
        std::swap(pieces, next);
    }
    return true;
}

double ExactBandIntegral::value(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Stats* stats) const
{
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    Timer t_all(cx, &Stats::t_total);
    Work w;
    if (!partition(T, w, stats)) return 0.;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const std::vector<Piece>& pieces = w.pieces;
    const Poly& Q0 = w.Q0;

    // steps 2-4: integrate each piece
    double total = 0., vol_sum = 0., empty_vol = 0., exact_vol = 0., rule_a_vol = 0.;
    int64_t rule_a_pieces = 0;
    std::vector<int> S;
    std::vector<const Loc*> members;
    for (const Piece& pc : pieces) {
        const double vol = volume_centroid(pc.Q).first;
        vol_sum += vol;
        if (pc.S.empty()) {
            empty_vol += vol;
            continue;
        }
        S = pc.S;
        if (S.size() > 1) S = prune_members(pc.Q, S, loc);
        bool all_faces = true, all_verts = true;
        members.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            all_faces = all_faces && loc[size_t(k)].kind == 0;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        double val;
        if (S.size() == 1) {
            val = int_g(*members[0], pc.Q, cx);
            exact_vol += vol;
        } else if (all_faces) {
            val = faces_min(pc.Q, members, cx);
            exact_vol += vol;
        } else if (all_verts) {
            val = vertices_min(pc.Q, members, cx);
            exact_vol += vol;
        } else { // rule (a): min over g of int d_g >= int min_g d_g, an upper bound
            val = std::numeric_limits<double>::infinity();
            for (const Loc* g : members) val = std::min(val, int_g(*g, pc.Q, cx));
            rule_a_vol += vol;
            ++rule_a_pieces;
        }
        total += val;
    }
    const double v0 = volume_centroid(Q0).first;
    // The two consistency checks, with bounds derived from the partition's own tolerances.
    //  - A clip that cuts R by the plane (g, h) builds both parts with vertices snapped within tol
    //    of the plane and rings cleaned at tol (max norm), so each part differs from the exact one
    //    only in the slab |g.x - h| <= sqrt(3) tol, whose section of R is at most A(R)/2 <= A(t)/2
    //    (R lies in t, and the surface area of nested convex bodies is monotone): the volume of
    //    the two parts moves by at most sqrt(3) tol A(t) per cutting clip.
    //  - In exact arithmetic the candidates' regions cover t, so a piece with no member exists only
    //    because a decision taken with a tolerance (a clip, or a region test) put a slab of
    //    half-width tol on the wrong side: at most tol A(t) of volume per decision.
    //  - Each volume is a fan sum of m triple products of vectors no longer than D, t's longest
    //    edge: its rounding is at most (13 m + m^2) eps D^3 / 6.
    // A lost or doubled piece or face exceeds these by orders of magnitude (MEASURED: a face kept
    // and capped twice made a sphere band cell 2.6e-6 over its volume, 84000 times its bound
    // 4.3e-16); the former fixed 1e-9 v0 did not scale with the cell's shape.
    double At = 0., D = 0.;
    for (int f = 0; f < Q0.faces(); ++f) {
        const Vec3* V = Q0.ring(f);
        At += 0.5 * (V[1] - V[0]).cross(V[2] - V[0]).norm();
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < i; ++j) D = std::max(D, (T[size_t(i)] - T[size_t(j)]).norm());
    }
    const auto rho = [&](const Poly& Q) {
        double m = 0.;
        for (int f = 0; f < Q.faces(); ++f) m += Q.size(f) - 2;
        return (13. * m + m * m) * EPS * D * D * D / 6.;
    };
    double rho_all = rho(Q0), rho_empty = 0.;
    for (const Piece& pc : pieces) {
        rho_all += rho(pc.Q);
        if (pc.S.empty()) rho_empty += rho(pc.Q);
    }
    const double tile_bound = std::sqrt(3.) * At * w.tol_cut + rho_all;
    const double empty_bound = At * (w.tol_any + std::sqrt(3.) * w.tol_cut) + rho_empty;
    if (stats) {
        ++stats->cells;
        stats->pieces += int64_t(pieces.size());
        stats->rule_a_pieces += rule_a_pieces;
        stats->vol += v0;
        stats->exact_vol += exact_vol;
        stats->rule_a_vol += rule_a_vol;
        stats->empty_vol += empty_vol;
    }
    if (!(empty_vol <= empty_bound)) {
        log_and_throw_error(
            "ExactBandIntegral: pieces outside every local region, volume {} of {} (bound {}) | {}",
            empty_vol,
            v0,
            empty_bound,
            cell_string(T, delta));
    }
    if (!(std::abs(vol_sum - v0) <= tile_bound)) {
        log_and_throw_error(
            "ExactBandIntegral: pieces do not tile t: {} vs {} (bound {}) | {}",
            vol_sum,
            v0,
            tile_bound,
            cell_string(T, delta));
    }
    return total - delta * v0;
}

double ExactBandIntegral::value_grad_hess(
    const std::array<Eigen::Vector3d, 4>& T,
    const int kv,
    const double delta,
    Eigen::Vector3d& G,
    Eigen::Matrix3d& H,
    Stats* stats) const
{
    G.setZero();
    H.setZero();
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    Timer t_all(cx, &Stats::t_total);
    Work w;
    if (!partition(T, w, stats)) return 0.;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;

    // elementary(): pieces on which the integrand is ONE smooth function. A face member is split
    // by its plane (affine per side); a rule-(a) piece carries the member of smallest integral
    // (lowest candidate on a tie), so its derivative is that of the BOUND.
    std::vector<Elem> pieces;
    Poly tmp;
    const auto add = [&](const Poly& Q, const Loc& g, const bool ra) {
        if (g.kind == 0) {
            for (const double sg : {1., -1.}) {
                const Cut c = clip(Q, -sg * g.dir, -sg * g.dir.dot(g.p), tmp, cx);
                if (c == Cut::None) continue;
                Elem e;
                e.Q = c == Cut::Same ? Q : tmp;
                e.kind = 0;
                e.dir = sg * g.dir;
                e.c = sg * g.dir.dot(g.p);
                e.rule_a = ra;
                pieces.push_back(std::move(e));
            }
            return;
        }
        Elem e;
        e.Q = Q;
        e.kind = g.kind;
        e.p = g.p;
        e.dir = g.dir;
        e.rule_a = ra;
        pieces.push_back(std::move(e));
    };
    std::vector<const Loc*> members;
    for (const Piece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        const std::vector<int> S = pc.S.size() > 1 ? prune_members(pc.Q, pc.S, loc) : pc.S;
        bool all_faces = true, all_verts = true;
        members.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            all_faces = all_faces && loc[size_t(k)].kind == 0;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        if (S.size() == 1) {
            add(pc.Q, loc[size_t(S[0])], false);
        } else if (all_faces) {
            faces_min_pieces(pc.Q, members, cx, [&](const Poly& R, const Vec3& W, const double c) {
                Elem e;
                e.Q = R;
                e.kind = 0;
                e.dir = W;
                e.c = c;
                pieces.push_back(std::move(e));
            });
        } else if (all_verts) {
            vertices_min_pieces(pc.Q, members, cx, [&](const Poly& R, const Vec3& v) {
                Elem e;
                e.Q = R;
                e.kind = 2;
                e.p = v;
                pieces.push_back(std::move(e));
            });
        } else {
            double best = std::numeric_limits<double>::infinity();
            int bk = -1;
            for (const int k : S) { // ascending k: the first minimum is the lowest on a tie
                const double v = int_g(loc[size_t(k)], pc.Q, cx);
                if (v < best) best = v, bk = k;
            }
            add(pc.Q, loc[size_t(bk)], true);
        }
    }

    // lambda0 = l0 + g0.y: 1 at x, 0 on the opposite face (face kv of tet_polytope is opposite
    // vertex kv)
    std::array<Vec3, 4> Tl;
    for (int i = 0; i < 4; ++i) Tl[size_t(i)] = T[size_t(i)] - w.c0;
    const Vec3 x = Tl[size_t(kv)];
    const Poly& Q0 = w.Q0;
    const Vec3* Vo = Q0.ring(kv);
    const Vec3& no = Q0.N[size_t(kv)];
    const double den = no.dot(x - Vo[0]);
    const Vec3 g0 = no / den;
    const double l0 = -no.dot(Vo[0]) / den;
    double scale = 0.;
    for (const Vec3& p : Tl) scale = std::max(scale, maxabs(p));
    const double tol = 64 * EPS * scale;
    std::array<std::pair<Vec3, double>, 4> tet_planes;
    for (int i = 0; i < 4; ++i) tet_planes[size_t(i)] = {Q0.N[size_t(i)], Q0.N[size_t(i)].dot(Q0.ring(i)[0])};
    struct TFace
    {
        int i;
        Vec3 nu;
        Eigen::Matrix3d S; // column k = e_k x (a - b)
        double nN; // |N_F|
    };
    std::vector<TFace> faces;
    Vec3 gradV = Vec3::Zero();
    for (int i = 0; i < 4; ++i) {
        if (i == kv) continue;
        const Vec3* V = Q0.ring(i);
        Vec3 ab[3];
        int nab = 0;
        for (int j = 0; j < 3; ++j) {
            if (maxabs(V[j] - x) > 0) ab[nab++] = V[j];
        }
        if (nab != 2) continue; // not reached: the face holds x and two other corners
        Vec3 a = ab[0], b = ab[1];
        Vec3 N = (x - b).cross(a - b);
        const Vec3& n = Q0.N[size_t(i)];
        if (N.dot(n) < 0) {
            std::swap(a, b);
            N = -N;
        }
        const Vec3 wab = a - b;
        TFace tf;
        tf.i = i;
        tf.nu = n;
        for (int k = 0; k < 3; ++k) tf.S.col(k) = Vec3::Unit(k).cross(wab);
        tf.nN = N.norm();
        faces.push_back(tf);
        gradV += n * N.norm() / 6.; // area / 3
    }

    double Jd = 0.;
    for (const Elem& e : pieces) {
        Jd += piece_value(e, cx);
        const Poly& Q = e.Q;
        // Rounding sliver (thickness = volume / largest face area at rounding level, e.g. the gap
        // between the input plane and a cell face lying on it): skipped. Its neighbours' cut
        // terms pair up across it, and the neighbour's face on the coincident plane is recognised
        // below as lying on the cell face (plane test, not the face's identity), so the trace
        // comes from the piece inside t.
        double amax = 0.;
        for (int f = 0; f < Q.faces(); ++f) {
            const Vec3* Vg = Q.ring(f);
            double s = 0.;
            for (int j = 1; j < Q.size(f) - 1; ++j) s += (Vg[j] - Vg[0]).cross(Vg[j + 1] - Vg[0]).norm();
            amax = std::max(amax, 0.5 * s);
        }
        if (volume_centroid(Q).first <= 1e3 * tol * amax) continue;
        for (int f = 0; f < Q.faces(); ++f) {
            const Vec3* Vf = Q.ring(f);
            const int kf = Q.size(f);
            const Vec3& nf = Q.N[size_t(f)];
            for (const TFace& tf : faces) {
                if (nf.dot(tf.nu) < 1 - 1e-12) continue;
                double dev = 0.;
                for (int j = 0; j < kf; ++j) {
                    dev = std::max(dev, std::abs(Vf[j].dot(tf.nu) - tet_planes[size_t(tf.i)].second));
                }
                if (dev > 1e3 * tol) continue; // same scale as the sliver test
                double P1;
                Vec3 P2;
                moments(Vf, kf, nf, e, l0, g0, P1, P2, cx);
                G += tf.nu * P1;
                H += tf.nu * P2.transpose() + tf.S * (P1 / tf.nN);
                for (int ee = 0; ee < kf; ++ee) {
                    const Vec3& y1 = Vf[ee];
                    const Vec3& y2 = Vf[(ee + 1) % kf];
                    if (maxabs(y2 - y1) <= tol) continue;
                    bool t_edge = false; // an edge of t: moves with it
                    for (int j = 0; j < 4 && !t_edge; ++j) {
                        const auto& [m, h] = tet_planes[size_t(j)];
                        t_edge = j != tf.i && std::abs(m.dot(y1) - h) <= tol && std::abs(m.dot(y2) - h) <= tol;
                    }
                    if (t_edge) continue;
                    // the neighbouring face of Q along this edge: the face whose plane holds both
                    // endpoints (its ring may carry extra collinear vertices on the edge); if
                    // several do (a zero-thickness sliver), the one traversing it backwards
                    int first = -1, ncand = 0;
                    Vec3 gc = Vec3::Zero();
                    for (int g = 0; g < Q.faces(); ++g) {
                        if (g == f) continue;
                        const Vec3& ng = Q.N[size_t(g)];
                        const double hg = ng.dot(Q.ring(g)[0]);
                        if (std::abs(ng.dot(y1) - hg) <= tol && std::abs(ng.dot(y2) - hg) <= tol) {
                            if (first < 0) first = g;
                            ++ncand;
                        }
                    }
                    if (first < 0) continue;
                    gc = Q.N[size_t(first)];
                    if (ncand > 1) {
                        for (int g = 0; g < Q.faces(); ++g) {
                            if (g == f) continue;
                            const Vec3& ng = Q.N[size_t(g)];
                            const Vec3* Vg = Q.ring(g);
                            const int kg = Q.size(g);
                            const double hg = ng.dot(Vg[0]);
                            if (!(std::abs(ng.dot(y1) - hg) <= tol && std::abs(ng.dot(y2) - hg) <= tol)) continue;
                            bool back = false;
                            for (int j = 0; j < kg && !back; ++j) {
                                back = maxabs(Vg[j] - y2) <= tol && maxabs(Vg[(j + 1) % kg] - y1) <= tol;
                            }
                            if (back) {
                                gc = ng;
                                break;
                            }
                        }
                    }
                    const Vec3 gp = gc - gc.dot(tf.nu) * tf.nu;
                    if (gp.norm() <= 1e-12) continue; // a plane parallel to F does not slide over it
                    const double P3 = cut_integral(y1, y2, e, l0, g0);
                    H -= tf.nu * gc.transpose() * (P3 / gp.norm());
                }
                break; // a polygon lies on at most one face of t
            }
        }
    }
    G = G - delta * gradV;
    return Jd - delta * volume_centroid(Q0).first;
}

} // namespace wmtk::components::topological_offset
