#include "segment_walk.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace wmtk::components::tracking {

using Eigen::Vector2d;
using Eigen::Vector3d;
using exact::Dyadic;

TriangleH lift(const Triangle2& t)
{
    return {{Vector3d(t[0].x(), t[0].y(), 1), Vector3d(t[1].x(), t[1].y(), 1),
             Vector3d(t[2].x(), t[2].y(), 1)}};
}

namespace exact {

namespace {

// Unit roundoff of IEEE double, round to nearest. The build has -ffp-contract=off, so every
// product and sum below is rounded on its own, as the bounds assume.
constexpr double kU = 0x1p-53;
constexpr double kU2 = 0x1p-106;
// Absolute error of one operation that underflows.
constexpr double kEta = 0x1p-1074;

thread_local Level t_level = kDouble;
thread_local bool t_undecided = false;
size_t g_exact = 0;

/// A bound computed in doubles, made safe against its own rounding.
double up(const double b)
{
    return b * (1 + 16 * kU) + 4 * kEta;
}

// ---- double-double arithmetic: a value hi + lo, |lo| <= ulp(hi) / 2.

struct DD
{
    double hi = 0, lo = 0;
};
DD two_sum(const double a, const double b)
{
    const double s = a + b;
    const double bb = s - a;
    return {s, (a - (s - bb)) + (b - bb)};
}
DD fast_two_sum(const double a, const double b)
{
    const double s = a + b;
    return {s, b - (s - a)};
}
DD two_prod(const double a, const double b)
{
    const double p = a * b;
    return {p, std::fma(a, b, -p)};
}
/// Within 3u^2 of a + b, relatively (Joldes, Muller, Popescu 2017, Algorithm 6).
DD add(const DD a, const DD b)
{
    const DD s = two_sum(a.hi, b.hi);
    const DD t = two_sum(a.lo, b.lo);
    const DD v = fast_two_sum(s.hi, s.lo + t.hi);
    return fast_two_sum(v.hi, t.lo + v.lo);
}
DD neg(const DD a)
{
    return {-a.hi, -a.lo};
}
/// Within 5u^2 of a b, relatively (ibid., Algorithm 11).
DD mul(const DD a, const DD b)
{
    const DD c = two_prod(a.hi, b.hi);
    return fast_two_sum(c.hi, c.lo + (a.hi * b.lo + a.lo * b.hi));
}
/// Within 2u^2 of a b, relatively (ibid., Algorithm 9).
DD mul(const DD a, const double b)
{
    const DD c = two_prod(a.hi, b);
    return fast_two_sum(c.hi, c.lo + a.lo * b);
}
/// Within 16u^2 of a / b, relatively.
DD div(const DD a, const DD b)
{
    const double q1 = a.hi / b.hi;
    const DD r = add(a, neg(mul(b, q1)));
    const double q2 = r.hi / b.hi;
    const DD r2 = add(r, neg(mul(b, q2)));
    return add(fast_two_sum(q1, q2), DD{r2.hi / b.hi, 0});
}
double magnitude(const DD a)
{
    return std::abs(a.hi) + std::abs(a.lo);
}

/// x rounded to a double towards zero, keeping x's sign even where that underflows to zero (the
/// sign of a value is exact), and the error of that.
double rounded(const Dyadic& x, double* error)
{
    const int s = x.sign();
    double v = x.to_double();
    if (v == 0 && s != 0) v = s * DBL_MIN;
    // Truncated: within one unit in the last place, 2u |v| (or DBL_MIN below that).
    *error = s == 0 ? 0. : std::max(2 * kU * std::abs(v), DBL_MIN);
    return v;
}

Value of_exact(const Dyadic& r)
{
    Value out;
    out.v = rounded(r, &out.e);
    if (out.e > 0) out.x = std::make_shared<const Dyadic>(r);
    return out;
}

/// p's coordinates translated by the column operation (x, y, w) -> (x - w o0, y - w o1, w), which
/// leaves every determinant of three points as it is but measures them from o, a corner near them:
/// the values then carry the rounding of differences, not of the coordinates themselves. With a
/// bound on their error, per coordinate, which includes that of the weights.
struct Approx
{
    Vector3d v = Vector3d::Zero();
    Vector3d e = Vector3d::Zero();
};

Approx approx(const Point& p, const Eigen::Vector2d& o)
{
    const TriangleH& t = *p.t;
    Approx a;
    Vector3d s = Vector3d::Zero();
    Vector3d from_weights = Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        if (p.w.v[k] == 0) continue;
        const Vector3d& c = t[k];
        const double ox = c[2] * o[0], oy = c[2] * o[1];
        const Vector3d d(c[0] - ox, c[1] - oy, c[2]);
        // The translated corner: within u of each difference, and of each product w o unless w is
        // 1 or 0 (exact).
        Vector3d ed(kU * std::abs(d[0]), kU * std::abs(d[1]), 0.);
        if (c[2] != 1 && c[2] != 0) {
            ed[0] += kU * std::abs(ox);
            ed[1] += kU * std::abs(oy);
        }
        // In doubles, a weight's low part is part of its error.
        const double w = p.w.v[k], ew = p.w.e[k] + std::abs(p.w.lo[k]);
        if (w == 1 && ew == 0 && p.w.zeros() == 2) {
            a.v = d;
            a.e = 1.0000001 * ed;
            return a;
        }
        a.v += w * d;
        s += std::abs(w) * (d.cwiseAbs() + ed);
        a.e += std::abs(w) * ed;
        // The weight itself is within ew of the exact one.
        from_weights += ew * (d.cwiseAbs() + ed);
    }
    // At most three products and two sums per coordinate: below 3u (1 + 3u) of s; s itself is
    // computed to within 3u of the exact sum of magnitudes.
    a.e = 1.0000001 * (a.e + from_weights) + 5 * kU * s;
    return a;
}

/// The same in double-doubles: the translated corners are exact (a difference or a product of
/// doubles is a double-double), and every operation on them within 5u^2 of its magnitude.
struct ApproxDD
{
    std::array<DD, 3> v;
    Vector3d mag = Vector3d::Zero(); // a bound on |v|
    Vector3d e = Vector3d::Zero();
};

ApproxDD approx_dd(const Point& p, const Eigen::Vector2d& o)
{
    const TriangleH& t = *p.t;
    ApproxDD a;
    Vector3d from_weights = Vector3d::Zero();
    Vector3d rounding = Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        if (p.w.v[k] == 0) continue;
        const Vector3d& c = t[k];
        std::array<DD, 3> d;
        for (int i = 0; i < 2; ++i) {
            // c_i - c_2 o_i: exact as the sum of two double-doubles, within 3u^2.
            d[i] = c[2] == 0 ? DD{c[i], 0} : add(DD{c[i], 0}, neg(two_prod(c[2], o[i])));
        }
        d[2] = DD{c[2], 0};
        const DD w{p.w.v[k], p.w.lo[k]};
        for (int i = 0; i < 3; ++i) {
            const DD wd = mul(w, d[i]);
            a.v[i] = add(a.v[i], wd);
            const double m = magnitude(w) * magnitude(d[i]);
            rounding[i] += m;
            from_weights[i] += p.w.e[k] * magnitude(d[i]);
        }
    }
    for (int i = 0; i < 3; ++i) {
        a.mag[i] = magnitude(a.v[i]);
        // The corners (3u^2), the products (5u^2) and the sums (3u^2 each) of the terms.
        a.e[i] = up(from_weights[i] + 16 * kU2 * rounding[i]);
    }
    return a;
}

/// A corner among the points' weighted corners to measure from: of c first, then b, then a, one
/// of the plane (w = 1); the origin if there is none.
Eigen::Vector2d origin(const Point& a, const Point& b, const Point& c)
{
    for (const Point* p : {&c, &b, &a}) {
        for (int k = 0; k < 3; ++k) {
            const Vector3d& x = (*p->t)[k];
            if (p->w.v[k] != 0 && x[2] == 1) return {x[0], x[1]};
        }
    }
    return Eigen::Vector2d::Zero();
}

double det3(const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
    return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
           a[2] * (b[0] * c[1] - b[1] * c[0]);
}

DD det3(const std::array<DD, 3>& a, const std::array<DD, 3>& b, const std::array<DD, 3>& c)
{
    const DD m0 = add(mul(b[1], c[2]), neg(mul(b[2], c[1])));
    const DD m1 = add(mul(b[0], c[2]), neg(mul(b[2], c[0])));
    const DD m2 = add(mul(b[0], c[1]), neg(mul(b[1], c[0])));
    return add(add(mul(a[0], m0), neg(mul(a[1], m1))), mul(a[2], m2));
}

/// The permanent of non-negative rows: what bounds det3's rounding.
double perm3(const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
    return a[0] * (b[1] * c[2] + b[2] * c[1]) + a[1] * (b[0] * c[2] + b[2] * c[0]) +
           a[2] * (b[0] * c[1] + b[1] * c[0]);
}

/// Whether the weights of a, b and c use at most two distinct corners: then the three points are
/// on the line through those (or on one point), and the determinant is zero exactly.
bool on_two_corners(const Point& a, const Point& b, const Point& c)
{
    const Vector3d* seen[2] = {nullptr, nullptr};
    int n = 0;
    for (const Point* p : {&a, &b, &c}) {
        for (int k = 0; k < 3; ++k) {
            if (p->w.v[k] == 0) continue;
            const Vector3d& x = (*p->t)[k];
            bool found = false;
            for (int i = 0; i < n; ++i) found = found || *seen[i] == x;
            if (found) continue;
            if (n == 2) return false;
            seen[n++] = &x;
        }
    }
    return true;
}

void exact_coordinates(const Point& p, Dyadic out[3])
{
    for (int i = 0; i < 3; ++i) out[i] = 0.;
    for (int k = 0; k < 3; ++k) {
        if (p.w.v[k] == 0) continue;
        const Dyadic w = p.w.x ? (*p.w.x)[k] : Dyadic(p.w.v[k]) + Dyadic(p.w.lo[k]);
        for (int i = 0; i < 3; ++i) {
            const double c = (*p.t)[k][i];
            if (c == 0) continue;
            out[i] = out[i] + (c == 1 ? w : w * Dyadic(c));
        }
    }
}

Dyadic exact_det(const Point& a, const Point& b, const Point& c)
{
    Dyadic x[3], y[3], z[3];
    exact_coordinates(a, x);
    exact_coordinates(b, y);
    exact_coordinates(c, z);
    return x[0] * (y[1] * z[2] - y[2] * z[1]) - x[1] * (y[0] * z[2] - y[2] * z[0]) +
           x[2] * (y[0] * z[1] - y[1] * z[0]);
}

/// The sign of det[a; b; c], and its value if `value` is given: in doubles if their bound decides
/// (and the precision asks no more), else in double-doubles if theirs does, else exactly.
int evaluate(const Point& a, const Point& b, const Point& c, Value* value)
{
    if (on_two_corners(a, b, c)) {
        if (value) *value = Value{};
        return 0;
    }
    const Level level = t_level;
    const Eigen::Vector2d o = origin(a, b, c);
    if (!value || level == kDouble) {
        const Approx A = approx(a, o), B = approx(b, o), C = approx(c, o);
        const double d = det3(A.v, B.v, C.v);
        const Vector3d pa = A.v.cwiseAbs(), pb = B.v.cwiseAbs(), pc = C.v.cwiseAbs();
        const double p0 = perm3(pa, pb, pc);
        const double pe = perm3(pa + A.e, pb + B.e, pc + C.e);
        // det3 of the rounded coordinates is within 5u (1 + 5u) p0 of their exact determinant,
        // and that is within pe - p0 of det[a; b; c] (det is multilinear and |det| <= perm); the
        // permanents themselves are computed to within 6u each.
        const double bound = up(24 * kU * pe + (pe - p0));
        // Coordinates that are not finite (a damaged input) have no exact value: no sign either.
        if (!std::isfinite(pe) || !std::isfinite(d)) {
            if (value) *value = Value{};
            return 0;
        }
        if (std::abs(d) > bound) {
            if (value) *value = Value{d, 0, bound, nullptr};
            return d > 0 ? 1 : -1;
        }
    }
    if (!value || level <= kDoubleDouble) {
        const ApproxDD A = approx_dd(a, o), B = approx_dd(b, o), C = approx_dd(c, o);
        const DD d = det3(A.v, B.v, C.v);
        // As above, the inputs' errors without cancellation: perm(A + eA, B + eB, C + eC) -
        // perm(A, B, C), telescoped into terms that are all non-negative; and the rounding of
        // twenty-odd double-double operations, each within 5u^2 of what it adds up.
        const Vector3d B1 = B.mag + B.e, C1 = C.mag + C.e;
        const double from_inputs =
            perm3(A.e, B1, C1) + perm3(A.mag, B.e, C1) + perm3(A.mag, B.mag, C.e);
        const double bound =
            up(from_inputs + 64 * kU2 * perm3(A.mag + A.e, B1, C1) + 4 * kU * kEta);
        if (std::abs(d.hi) > bound + std::abs(d.lo) && std::isfinite(d.hi)) {
            if (value) *value = Value{d.hi, d.lo, bound, nullptr};
            return d.hi > 0 ? 1 : -1;
        }
    }
    if (!(a.w.known() && b.w.known() && c.w.known())) {
        give_up();
        if (value) *value = Value{};
        return 0;
    }
    ++g_exact;
    const Dyadic r = exact_det(a, b, c);
    if (value) *value = of_exact(r);
    return r.sign();
}

} // namespace

Hom Hom::of(const Eigen::Vector3d& v)
{
    Hom h;
    h.v = v;
    return h;
}

Exact3 Hom::value() const
{
    if (x) return *x;
    return {Dyadic(v[0]) + Dyadic(lo[0]), Dyadic(v[1]) + Dyadic(lo[1]), Dyadic(v[2]) + Dyadic(lo[2])};
}

Dyadic Value::value() const
{
    if (x) return *x;
    return Dyadic(v) + Dyadic(lo);
}

bool same(const Hom& a, const Hom& b)
{
    if (a.known() && b.known()) {
        if (!a.x && !b.x && a.v == b.v && a.lo == b.lo) return true;
        // Homogeneous: the same point if x sum(y) = y sum(x).
        const Exact3 x = a.value(), y = b.value();
        const Dyadic sx = x[0] + x[1] + x[2], sy = y[0] + y[1] + y[2];
        for (int k = 0; k < 3; ++k) {
            if (x[k] * sy != y[k] * sx) return false;
        }
        return true;
    }
    // Coordinates within each other's bounds may or may not be the same point: two points
    // computed apart are told apart only by their exact coordinates.
    for (int k = 0; k < 3; ++k) {
        const DD g = add(two_sum(a.v[k], -b.v[k]), two_sum(a.lo[k], -b.lo[k]));
        const double gap = std::abs(g.hi) - std::abs(g.lo);
        if (gap > up(a.e[k] + b.e[k] + 4 * kU2 * (std::abs(a.v[k]) + std::abs(b.v[k])))) {
            return false;
        }
    }
    give_up();
    return false;
}

Value coordinate(const Hom& h, const int k)
{
    Value out{h.v[k], h.lo[k], h.e[k], nullptr};
    if (h.x) out.x = std::make_shared<const Dyadic>((*h.x)[k]);
    return out;
}

Hom hom(const Value& a, const Value& b, const Value& c)
{
    Hom h;
    h.v = Vector3d(a.v, b.v, c.v);
    h.lo = Vector3d(a.lo, b.lo, c.lo);
    h.e = Vector3d(a.e, b.e, c.e);
    if ((a.x || b.x || c.x) && a.known() && b.known() && c.known()) {
        h.x = std::make_shared<const Exact3>(Exact3{a.value(), b.value(), c.value()});
    }
    return h;
}

Value negated(Value v)
{
    v.v = -v.v;
    v.lo = -v.lo;
    if (v.x) v.x = std::make_shared<const Dyadic>(-*v.x);
    return v;
}

Hom normalized(const Hom& h)
{
    if (h.zeros() == 2) {
        // A corner: (1, 0, 0) exactly, whatever the scale -- one representation per corner.
        Hom out;
        out.v[h.v[0] != 0 ? 0 : (h.v[1] != 0 ? 1 : 2)] = 1;
        return out;
    }
    if (h.x || (t_level == kExact && h.known())) {
        // Exactly: scaled by a power of two to a sum between 1 and 2, which divides by nothing.
        Exact3 r = h.value();
        const Dyadic s = r[0] + r[1] + r[2];
        const long k = -s.ilogb();
        Hom out;
        for (int i = 0; i < 3; ++i) {
            if (h.v[i] == 0) continue;
            r[i] = r[i].scaled(k);
            out.v[i] = rounded(r[i], &out.e[i]);
        }
        out.x = std::make_shared<const Exact3>(std::move(r));
        return out;
    }
    const double E = h.e.sum();
    Hom out;
    if (t_level == kDoubleDouble) {
        DD s;
        for (int k = 0; k < 3; ++k) s = add(s, DD{h.v[k], h.lo[k]});
        // s is within 6u^2 s of the exact sum of v + lo, and that within E of the exact sum.
        const double S = s.hi - up(E + 8 * kU2 * s.hi + std::abs(s.lo));
        if (!(S > 0)) {
            give_up();
            return h;
        }
        for (int k = 0; k < 3; ++k) {
            if (h.v[k] == 0) continue;
            const DD q = div(DD{h.v[k], h.lo[k]}, s);
            out.v[k] = q.hi;
            out.lo[k] = q.lo;
            out.e[k] = up((h.e[k] + magnitude(q) * (E + 8 * kU2 * s.hi)) / S + 32 * kU2 * magnitude(q));
        }
        return out;
    }
    // In doubles, the low parts are part of the error.
    const double s = h.v.sum();
    const double El = E + h.lo.cwiseAbs().sum();
    // s is within 2u s of the exact sum of v (the coordinates are of one sign), and that within E
    // of the exact sum; each quotient is rounded once.
    const double S = s - up(El + 2 * kU * s);
    if (!(S > 0)) {
        give_up();
        return h;
    }
    for (int k = 0; k < 3; ++k) {
        if (h.v[k] == 0) continue;
        out.v[k] = h.v[k] / s;
        out.e[k] = up(
            (h.e[k] + std::abs(h.lo[k]) + std::abs(out.v[k]) * (El + 2 * kU * s)) / S +
            kU * std::abs(out.v[k]));
    }
    return out;
}

Precision::Precision(const Level level)
    : m_previous(t_level)
{
    t_level = level;
}
Precision::~Precision()
{
    t_level = m_previous;
}
Level precision()
{
    return t_level;
}

bool undecided()
{
    return t_undecided;
}
void clear_undecided()
{
    t_undecided = false;
}
void give_up()
{
    t_undecided = true;
}

Point corner(const TriangleH& t, const int k)
{
    Point p{&t, Hom{}};
    p.w.v[k] = 1;
    return p;
}

int orient(const Point& a, const Point& b, const Point& c)
{
    return evaluate(a, b, c, nullptr);
}

Value det(const Point& a, const Point& b, const Point& c)
{
    Value v;
    evaluate(a, b, c, &v);
    return v;
}

Value lin2(const Value& a, const double b, const Value& c, const double d)
{
    const Level level = t_level;
    if (level == kDouble) {
        const double p = a.v * b, q = c.v * d;
        const double ep = std::fma(a.v, b, -p), eq = std::fma(c.v, d, -q);
        // a.v b - c.v d = (p - q) + (ep - eq) exactly; each difference is rounded once.
        const double s1 = p - q, s2 = ep - eq;
        const double r = s1 + s2;
        if (!std::isfinite(r)) return Value{};
        // Error: at most u |s1| + u |s2| from the differences and u |r| from the sum, and that of
        // the inputs (their low parts included).
        const double bound = up(4 * kU * (std::abs(s1) + std::abs(s2)) +
                                std::abs(b) * (a.e + std::abs(a.lo)) +
                                std::abs(d) * (c.e + std::abs(c.lo)));
        if (std::abs(r) > bound) return Value{r, 0, bound, nullptr};
    }
    if (level <= kDoubleDouble) {
        const DD r = add(mul(DD{a.v, a.lo}, b), neg(mul(DD{c.v, c.lo}, d)));
        const double bound = up(std::abs(b) * a.e + std::abs(d) * c.e +
                                8 * kU2 * (magnitude(DD{a.v, a.lo}) * std::abs(b) +
                                           magnitude(DD{c.v, c.lo}) * std::abs(d)));
        if (std::isfinite(r.hi) && std::abs(r.hi) > bound + std::abs(r.lo)) {
            return Value{r.hi, r.lo, bound, nullptr};
        }
    }
    if (!a.known() || !c.known()) {
        give_up();
        return Value{};
    }
    ++g_exact;
    return of_exact(a.value() * Dyadic(b) - c.value() * Dyadic(d));
}

Value muladd(const Value& a, const Value& b, const double c)
{
    if (b.v == 0 || c == 0) return a;
    if (a.v == 0 && c == 1) return b;
    // Terms of one sign, one of them non-zero, have a sum of that sign.
    const bool one_sign = (a.v >= 0 && b.v * c >= 0) || (a.v <= 0 && b.v * c <= 0);
    const Level level = t_level;
    if (level == kDouble) {
        const double p = b.v * c;
        const double r = a.v + p;
        const double bound = up(kU * std::abs(p) + kU * std::abs(r) + a.e + std::abs(a.lo) +
                                std::abs(c) * (b.e + std::abs(b.lo)));
        if (std::isfinite(r) && r != 0 && (std::abs(r) > bound || one_sign)) {
            return Value{r, 0, bound, nullptr};
        }
    }
    if (level <= kDoubleDouble) {
        const DD p = mul(DD{b.v, b.lo}, c);
        const DD r = add(DD{a.v, a.lo}, p);
        const double bound =
            up(a.e + std::abs(c) * b.e + 8 * kU2 * (magnitude(DD{a.v, a.lo}) + magnitude(p)));
        if (std::isfinite(r.hi) && r.hi != 0 && (std::abs(r.hi) > bound + std::abs(r.lo) || one_sign)) {
            return Value{r.hi, r.lo, bound, nullptr};
        }
    }
    if (!a.known() || !b.known()) {
        give_up();
        return Value{};
    }
    return of_exact(a.value() + b.value() * Dyadic(c));
}

size_t exact_evaluations()
{
    return g_exact;
}

} // namespace exact

namespace {

using exact::Hom;

template <typename T>
int corner_of(const std::array<T, 3>& t, const T& c)
{
    for (int k = 0; k < 3; ++k) {
        if (t[k] == c) return k;
    }
    return -1;
}

/// The triangle holding the edge from a to b, in that direction.
std::optional<size_t>
across(const std::vector<TriangleH>& tris, const Vector3d& a, const Vector3d& b)
{
    for (size_t n = 0; n < tris.size(); ++n) {
        const int i = corner_of(tris[n], a);
        if (i >= 0 && tris[n][(i + 1) % 3] == b) return n;
    }
    return std::nullopt;
}

Hom unit(const int k)
{
    Hom l;
    l.v[k] = 1;
    return l;
}

} // namespace

std::optional<Hom> carry(const TriangleH& t, const Hom& l, const TriangleH& n)
{
    Hom out;
    out.stored = l.stored;
    std::array<int, 3> to = {{-1, -1, -1}};
    for (int k = 0; k < 3; ++k) {
        if (l.v[k] == 0) continue;
        const int c = corner_of(n, t[k]);
        if (c < 0) return std::nullopt;
        to[k] = c;
        out.v[c] = l.v[k];
        out.lo[c] = l.lo[k];
        out.e[c] = l.e[k];
    }
    if (l.x) {
        exact::Exact3 x;
        for (int k = 0; k < 3; ++k) {
            if (to[k] >= 0) x[to[k]] = (*l.x)[k];
        }
        out.x = std::make_shared<const exact::Exact3>(std::move(x));
    }
    return out;
}

std::optional<Vector3d> carry(const TriangleH& t, const Vector3d& l, const TriangleH& n)
{
    Vector3d out = Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        if (l[k] == 0) continue;
        const int c = corner_of(n, t[k]);
        if (c < 0) return std::nullopt;
        out[c] = l[k];
    }
    return out;
}

std::optional<Vector3d> carry(const Triangle2& t, const Vector3d& l, const Triangle2& n)
{
    return carry(lift(t), l, lift(n));
}

std::optional<HWalkPoint> locate(const std::vector<TriangleH>& tris, const exact::Point& p)
{
    using exact::corner;
    for (size_t k = 0; k < tris.size(); ++k) {
        const TriangleH& t = tris[k];
        // The homogeneous barycentric coordinates of p: by Cramer's rule, up to det t > 0.
        const exact::Point c0 = corner(t, 0), c1 = corner(t, 1), c2 = corner(t, 2);
        const int s0 = exact::orient(p, c1, c2);
        if (exact::undecided()) return std::nullopt;
        if (s0 < 0) continue;
        const int s1 = exact::orient(c0, p, c2);
        if (exact::undecided()) return std::nullopt;
        if (s1 < 0) continue;
        const int s2 = exact::orient(c0, c1, p);
        if (exact::undecided()) return std::nullopt;
        if (s2 < 0) continue;
        if (s0 == 0 && s1 == 0 && s2 == 0) continue; // a degenerate triangle
        const exact::Value zero;
        const Hom l = exact::hom(
            s0 ? exact::det(p, c1, c2) : zero,
            s1 ? exact::det(c0, p, c2) : zero,
            s2 ? exact::det(c0, c1, p) : zero);
        return HWalkPoint{k, exact::normalized(l)};
    }
    return std::nullopt;
}

std::optional<std::vector<HWalkPiece>>
walk(const std::vector<TriangleH>& tris, const HWalkPoint& from, const HWalkPoint& to)
{
    using exact::corner;
    using exact::orient;
    std::vector<HWalkPiece> pieces;
    if (from.tri == to.tri) {
        if (!exact::same(from.bary, to.bary)) pieces.push_back({from.tri, {from.bary, to.bary}});
        return pieces;
    }
    // The line is decided by the two ends as given; it is the same line from start to end.
    const exact::Point P{&tris[from.tri], from.bary};
    const exact::Point Q{&tris[to.tri], to.bary};
    // Which side of the line from P to Q a corner is on: 1 left, -1 right, 0 on it.
    const auto side = [&](const TriangleH& t, const int k) { return orient(P, Q, corner(t, k)); };

    size_t t = from.tri;
    Hom at = from.bary;
    // A start on an edge is on two triangles: take the one it goes into.
    if (at.zeros() == 1) {
        const int z = at.v[0] == 0 ? 0 : (at.v[1] == 0 ? 1 : 2);
        const TriangleH& tri = tris[t];
        if (orient(corner(tri, (z + 1) % 3), corner(tri, (z + 2) % 3), Q) < 0) {
            if (const auto n = across(tris, tri[(z + 2) % 3], tri[(z + 1) % 3])) {
                if (const auto l = carry(tri, at, tris[*n])) {
                    t = *n;
                    at = *l;
                }
            }
        }
    }

    // Whether the end is in triangle t: then the rest of the segment is the piece from `at` to it.
    const auto reached = [&]() {
        if (t == to.tri) {
            if (!exact::same(at, to.bary)) pieces.push_back({t, {at, to.bary}});
            return true;
        }
        // The end is on a corner or an edge this triangle has too.
        if (const auto end = carry(tris[to.tri], to.bary, tris[t])) {
            if (!exact::same(at, *end)) pieces.push_back({t, {at, *end}});
            return true;
        }
        return false;
    };
    // A straight walk crosses each triangle once, so more steps than this mean an inverted one.
    for (size_t step = 0; step < 4 * tris.size() + 8; ++step) {
        if (exact::undecided()) return std::nullopt;
        if (reached()) return pieces;
        // On a corner, any of the triangles around it may be the one it goes on through.
        if (at.zeros() == 2) {
            const int k = at.v[0] != 0 ? 0 : (at.v[1] != 0 ? 1 : 2);
            const Vector3d v = tris[t][k];
            // Straight on to a corner of the last triangle: the rest is inside it.
            if (const int j = corner_of(tris[to.tri], v); j >= 0) {
                t = to.tri;
                at = unit(j);
                continue;
            }
            // The triangle (v, a, b) the line goes into: a on its right and b on its left, or
            // along an edge from v -- a on the line with b on the left, or b on it with a on the
            // right (the corner on the line is then ahead of v, as det[v, a, b] > 0 says).
            bool found = false;
            for (size_t n = 0; n < tris.size() && !found; ++n) {
                const int j = corner_of(tris[n], v);
                if (j < 0) continue;
                const int sa = side(tris[n], (j + 1) % 3), sb = side(tris[n], (j + 2) % 3);
                if (sa <= 0 && sb >= 0 && !(sa == 0 && sb == 0)) {
                    t = n;
                    at = unit(j);
                    found = true;
                }
            }
            if (!found) return std::nullopt;
            if (reached()) return pieces;
        }

        const TriangleH& tri = tris[t];
        const std::array<int, 3> s = {{side(tri, 0), side(tri, 1), side(tri, 2)}};
        // Leaving through an edge: its start on the right of the line, its end on the left.
        int edge = -1;
        for (int i = 0; i < 3; ++i) {
            if (s[i] < 0 && s[(i + 1) % 3] > 0) edge = i;
        }
        if (edge >= 0) {
            const int a = edge, b = (edge + 1) % 3;
            const auto n = across(tris, tri[b], tri[a]);
            if (!n) return std::nullopt;
            // The line meets the edge at det[P, Q, b] A - det[P, Q, a] B: two positive weights,
            // so the crossing is strictly inside the edge.
            const exact::Value ra = exact::negated(exact::det(P, Q, corner(tri, a)));
            const exact::Value rb = exact::det(P, Q, corner(tri, b));
            std::array<exact::Value, 3> w;
            w[a] = rb;
            w[b] = ra;
            const Hom next = exact::normalized(exact::hom(w[0], w[1], w[2]));
            pieces.push_back({t, {at, next}});
            const auto l = carry(tri, next, tris[*n]);
            if (!l) return std::nullopt;
            t = *n;
            at = *l;
            continue;
        }
        // Leaving through a corner: one on the line, ahead of where the walk is. With R a corner
        // off the line, X is ahead of `at` iff det[R, at, X] has the sign of det[P, Q, R].
        int r = -1;
        for (int k = 0; k < 3; ++k) {
            if (s[k] != 0) r = k;
        }
        if (r < 0) return std::nullopt;
        const exact::Point here{&tri, at};
        int corner_out = -1;
        for (int k = 0; k < 3; ++k) {
            if (s[k] != 0 || (at.zeros() == 2 && at.v[k] != 0)) continue;
            if (orient(corner(tri, r), here, corner(tri, k)) == s[r]) corner_out = k;
        }
        if (corner_out < 0) return std::nullopt;
        pieces.push_back({t, {at, unit(corner_out)}});
        at = unit(corner_out);
    }
    return std::nullopt;
}

std::optional<std::vector<WalkPiece>>
walk(const std::vector<TriangleH>& tris, const WalkPoint& from, const WalkPoint& to)
{
    const HWalkPoint a{from.tri, Hom::of(from.bary)}, b{to.tri, Hom::of(to.bary)};
    exact::clear_undecided();
    auto pieces = walk(tris, a, b);
    if (exact::undecided()) {
        exact::clear_undecided();
        const exact::Precision exactly(exact::kExact);
        pieces = walk(tris, a, b);
    }
    if (!pieces) return std::nullopt;
    std::vector<WalkPiece> out;
    out.reserve(pieces->size());
    for (const HWalkPiece& p : *pieces) out.push_back({p.tri, {p.bary[0].v, p.bary[1].v}});
    return out;
}

std::optional<std::vector<WalkPiece>>
walk_segment(const std::vector<Triangle2>& tris, const WalkPoint& from, const WalkPoint& to)
{
    std::vector<TriangleH> lifted;
    lifted.reserve(tris.size());
    for (const Triangle2& t : tris) lifted.push_back(lift(t));
    return walk(lifted, from, to);
}

} // namespace wmtk::components::tracking
