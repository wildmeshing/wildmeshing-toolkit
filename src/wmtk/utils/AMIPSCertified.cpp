// AMIPS_energy_p3_certified: the exact AMIPS^3 fallback of AMIPS_energy_stable_p3, without GMP
// when it can be certified. See the declaration in AMIPS.h.
//
// Identity (checked exactly over random rational tets): AMIPS_energy_rational_p3(T) is
//     27/16 * S^3 / tmp^2 = S3^3 / (16 D^2),
// with S3 the sum of the squared lengths of the six edges, S = S3/3, and D = det[p1-p0, p2-p0,
// p3-p0] = -tmp; the GMP path returns mpq_get_d of it, which truncates toward zero.
//
// Method:
//   D   exactly, as a Shewchuk expansion of the (1- or 2-term, exact) coordinate differences,
//       compressed; its double-double value is the top two components, and the remainder is
//       bounded at run time.
//   S3  in double-double: a sum of 18 non-negative squares, so a forward error bound holds.
//   E   = S3^3 / (16 D^2) in double-double, every operation with a known error bound
//       (Joldes-Muller-Popescu 2017 DWTimesDW3, a Newton-corrected division): in total below
//       about 2^-92 relative.
// The result is certified when the window E * (1 +- 2^-80) holds no double, so trunc(E) is known
// exactly -- GMP's answer bit for bit. Otherwise (E within 2^-80 of a double, e.g. exactly one),
// or outside the exponent range the bounds assume, the caller uses the GMP path.
//
// Error-free transformations: this file must be compiled with IEEE double arithmetic. Any fused
// multiply-add is written out (std::fma), and contraction of the rest could only make some steps
// more accurate, which keeps every bound -- it can change whether a value is certified, never
// the value returned.
#include "AMIPS.h"

#include <array>
#include <cmath>
#include <cstdint>

#if defined(__FAST_MATH__)
#error \
    "AMIPSCertified.cpp relies on IEEE arithmetic; -ffast-math breaks its error-free transformations"
#endif
#if defined(__clang__)
#pragma STDC FP_CONTRACT OFF
#endif

namespace wmtk {
namespace {

inline void two_sum(double a, double b, double& s, double& e)
{
    s = a + b;
    const double bv = s - a;
    const double av = s - bv;
    e = (a - av) + (b - bv);
}
inline void fast_two_sum(double a, double b, double& s, double& e) // |a| >= |b| or a == 0
{
    s = a + b;
    e = b - (s - a);
}
inline void two_diff(double a, double b, double& d, double& e)
{
    d = a - b;
    const double bv = a - d;
    const double av = d + bv;
    e = (a - av) + (bv - b);
}
inline void two_prod(double a, double b, double& p, double& e)
{
    p = a * b;
    e = std::fma(a, b, -p);
}

// Shewchuk, zero-eliminating. h must not alias e/f.
inline int scale_expansion(int elen, const double* e, double b, double* h)
{
    double Q, hh, p1, p0, sum;
    int hi = 0;
    two_prod(e[0], b, Q, hh);
    if (hh != 0.0) h[hi++] = hh;
    for (int i = 1; i < elen; ++i) {
        two_prod(e[i], b, p1, p0);
        two_sum(Q, p0, sum, hh);
        if (hh != 0.0) h[hi++] = hh;
        fast_two_sum(p1, sum, Q, hh);
        if (hh != 0.0) h[hi++] = hh;
    }
    if (Q != 0.0 || hi == 0) h[hi++] = Q;
    return hi;
}

inline int expansion_sum(int elen, const double* e, int flen, const double* f, double* h)
{
    // fast_expansion_sum_zeroelim, with bounds-checked reads.
    double Q, Qnew, hh;
    int ei = 0, fi = 0, hi = 0;
    double en = e[0], fn = f[0];
    if ((fn > en) == (fn > -en)) {
        Q = en;
        ++ei;
        en = ei < elen ? e[ei] : 0.0;
    } else {
        Q = fn;
        ++fi;
        fn = fi < flen ? f[fi] : 0.0;
    }
    if (ei < elen && fi < flen) {
        if ((fn > en) == (fn > -en)) {
            fast_two_sum(en, Q, Qnew, hh);
            ++ei;
            en = ei < elen ? e[ei] : 0.0;
        } else {
            fast_two_sum(fn, Q, Qnew, hh);
            ++fi;
            fn = fi < flen ? f[fi] : 0.0;
        }
        Q = Qnew;
        if (hh != 0.0) h[hi++] = hh;
        while (ei < elen && fi < flen) {
            if ((fn > en) == (fn > -en)) {
                two_sum(Q, en, Qnew, hh);
                ++ei;
                en = ei < elen ? e[ei] : 0.0;
            } else {
                two_sum(Q, fn, Qnew, hh);
                ++fi;
                fn = fi < flen ? f[fi] : 0.0;
            }
            Q = Qnew;
            if (hh != 0.0) h[hi++] = hh;
        }
    }
    while (ei < elen) {
        two_sum(Q, en, Qnew, hh);
        ++ei;
        en = ei < elen ? e[ei] : 0.0;
        Q = Qnew;
        if (hh != 0.0) h[hi++] = hh;
    }
    while (fi < flen) {
        two_sum(Q, fn, Qnew, hh);
        ++fi;
        fn = fi < flen ? f[fi] : 0.0;
        Q = Qnew;
        if (hh != 0.0) h[hi++] = hh;
    }
    if (Q != 0.0 || hi == 0) h[hi++] = Q;
    return hi;
}

// Shewchuk compress; in place is fine (e == h allowed).
inline int compress(int elen, double* e, double* h)
{
    int bottom = elen - 1;
    double Q = e[bottom], Qnew, q;
    for (int i = elen - 2; i >= 0; --i) {
        fast_two_sum(Q, e[i], Qnew, q);
        if (q != 0.0) {
            e[bottom--] = Qnew;
            Q = q;
        } else {
            Q = Qnew;
        }
    }
    int top = 0;
    for (int i = bottom + 1; i < elen; ++i) {
        fast_two_sum(e[i], Q, Qnew, q);
        if (q != 0.0) h[top++] = q;
        Q = Qnew;
    }
    h[top] = Q;
    return top + 1;
}

// a 1- or 2-component expansion (exact difference)
struct E2
{
    double c[2];
    int n;
};
inline E2 exact_diff(double a, double b)
{
    E2 r;
    double d, e;
    two_diff(a, b, d, e);
    if (e == 0.0) {
        r.c[0] = d;
        r.n = 1;
    } else {
        r.c[0] = e;
        r.c[1] = d;
        r.n = 2;
    }
    return r;
}
// h = x * y for short expansions; returns length
inline int mul_e2(const E2& x, const E2& y, double* h)
{
    if (y.n == 1) return scale_expansion(x.n, x.c, y.c[0], h);
    double t0[4], t1[4];
    const int n0 = scale_expansion(x.n, x.c, y.c[0], t0);
    const int n1 = scale_expansion(x.n, x.c, y.c[1], t1);
    return expansion_sum(n0, t0, n1, t1, h);
}
inline int mul_ex(int n, const double* x, const E2& y, double* h, double* tmp)
{
    if (y.n == 1) return scale_expansion(n, x, y.c[0], h);
    const int n0 = scale_expansion(n, x, y.c[0], tmp);
    const int n1 = scale_expansion(n, x, y.c[1], tmp + 2 * n);
    return expansion_sum(n0, tmp, n1, tmp + 2 * n, h);
}
inline void negate(int n, double* x)
{
    for (int i = 0; i < n; ++i) x[i] = -x[i];
}

struct dd
{
    double h, l;
};
// Joldes-Muller-Popescu 2017, Alg. 12 (DWTimesDW3): rel. error <= 5u^2 (we budget 7u^2).
inline dd dd_mul(dd x, dd y)
{
    double ch, cl1;
    two_prod(x.h, y.h, ch, cl1);
    const double tl0 = x.l * y.l;
    const double tl1 = std::fma(x.h, y.l, tl0);
    const double cl2 = std::fma(x.l, y.h, tl1);
    const double cl3 = cl1 + cl2;
    dd z;
    fast_two_sum(ch, cl3, z.h, z.l);
    return z;
}
// x / y, y > 0, both normalized: one Newton correction. rel. error <= ~6u^2 (budget 16u^2).
inline dd dd_div(dd x, dd y)
{
    const double t = x.h / y.h;
    double ph, pl;
    two_prod(t, y.h, ph, pl); // t*y.h exactly
    const double m = t * y.l;
    const double r0 = x.h - ph; // exact (Sterbenz: ph within 2u of x.h)
    const double r1 = x.l - pl;
    const double r2 = r1 - m;
    const double r = r0 + r2;
    const double c = r / y.h;
    dd z;
    fast_two_sum(t, c, z.h, z.l);
    return z;
}

enum class Status : int { certified = 0, zero_det = 1, uncertified = 2, out_of_range = 3 };

// Returns certified/zero_det with `out` set (inf for zero_det) -- identical to the GMP path --
// or uncertified/out_of_range, in which case the caller runs the exact GMP path.
inline Status amips_p3(const std::array<double, 12>& T, double& out)
{
    // Range guard: every coordinate finite, every non-zero difference and tail in a range where
    // no product of three of them (and its error terms) can under- or overflow.
    // ---- D = det[p1-p0, p2-p0, p3-p0] exactly
    E2 a[3], b[3], c[3];
    double maxabs = 0.0, minabs = INFINITY;
    for (int k = 0; k < 3; ++k) {
        a[k] = exact_diff(T[3 + k], T[k]);
        b[k] = exact_diff(T[6 + k], T[k]);
        c[k] = exact_diff(T[9 + k], T[k]);
    }
    for (int k = 0; k < 3; ++k) {
        for (const E2* v : {&a[k], &b[k], &c[k]}) {
            for (int i = 0; i < v->n; ++i) {
                const double m = std::fabs(v->c[i]);
                if (!(m <= 0x1p100)) return Status::out_of_range; // also NaN / inf
                if (m != 0.0 && m < minabs) minabs = m;
                if (m > maxabs) maxabs = m;
            }
        }
    }
    if (minabs < 0x1p-280) return Status::out_of_range;

    double m1[8], m2[8], mm[16], t1[64], t2[64], t3[64], s12[128], dexp[192], tmp[64];
    int n1, n2, nm, nt1, nt2, nt3;
    // minor_x = b_y c_z - b_z c_y
    n1 = mul_e2(b[1], c[2], m1);
    n2 = mul_e2(b[2], c[1], m2);
    negate(n2, m2);
    nm = expansion_sum(n1, m1, n2, m2, mm);
    nt1 = mul_ex(nm, mm, a[0], t1, tmp);
    // minor_y = b_z c_x - b_x c_z
    n1 = mul_e2(b[2], c[0], m1);
    n2 = mul_e2(b[0], c[2], m2);
    negate(n2, m2);
    nm = expansion_sum(n1, m1, n2, m2, mm);
    nt2 = mul_ex(nm, mm, a[1], t2, tmp);
    // minor_z = b_x c_y - b_y c_x
    n1 = mul_e2(b[0], c[1], m1);
    n2 = mul_e2(b[1], c[0], m2);
    negate(n2, m2);
    nm = expansion_sum(n1, m1, n2, m2, mm);
    nt3 = mul_ex(nm, mm, a[2], t3, tmp);
    const int ns12 = expansion_sum(nt1, t1, nt2, t2, s12);
    int nd = expansion_sum(ns12, s12, nt3, t3, dexp);
    nd = compress(nd, dexp, dexp);
    if (dexp[nd - 1] == 0.0) {
        out = INFINITY; // GMP: tmp == 0 -> +inf
        return Status::zero_det;
    }
    dd d;
    double rem = 0.0;
    if (nd == 1) {
        d = {dexp[0], 0.0};
    } else {
        two_sum(dexp[nd - 1], dexp[nd - 2], d.h, d.l);
        if (nd >= 3) rem = 2.0 * std::fabs(dexp[nd - 3]); // > |sum of the rest| (non-overlapping)
    }
    if (d.h < 0) {
        d.h = -d.h;
        d.l = -d.l;
    }
    if (!(rem <= std::ldexp(d.h, -95))) return Status::uncertified; // never seen in practice

    // ---- S3 = sum_{i<j} |p_i - p_j|^2 in double-double, all terms >= 0
    // hi parts summed with a compensated (two_sum) cascade, lo parts in plain double.
    double sh = 0.0, sl = 0.0;
    static constexpr int P[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (int e = 0; e < 6; ++e) {
        for (int k = 0; k < 3; ++k) {
            double h, l, p, pe;
            two_diff(T[3 * P[e][0] + k], T[3 * P[e][1] + k], h, l);
            two_prod(h, h, p, pe);
            const double ll = l * l;
            const double t = std::fma(2.0 * h, l, ll); // 2hl + l^2
            const double q = pe + t;
            double s, se;
            two_sum(sh, p, s, se);
            sh = s;
            sl += se;
            sl += q;
        }
    }
    dd s;
    fast_two_sum(sh, sl, s.h, s.l);
    if (!(s.h >= 0x1p-280 && s.h <= 0x1p240)) return Status::out_of_range;
    if (!(d.h >= 0x1p-420 && d.h <= 0x1p360)) return Status::out_of_range;

    // ---- E = s^3 / (16 d^2)
    const dd s2 = dd_mul(s, s);
    const dd s3 = dd_mul(s2, s);
    dd d2 = dd_mul(d, d);
    d2.h *= 16.0;
    d2.l *= 16.0;
    const dd e = dd_div(s3, d2);

    // ---- certify trunc(E)
    if (!(e.h >= 26.0 && e.h < 0x1p1000)) return Status::out_of_range;
    const double w2 = std::ldexp(e.h, -79); // 2 * 2^-80 * e.h, exact
    if (e.l > w2) {
        out = e.h;
        return Status::certified;
    }
    if (e.l < -w2) {
        const double prev = std::nextafter(e.h, 0.0);
        const double gap = e.h - prev; // exact
        if (e.l + gap > w2) {
            out = prev;
            return Status::certified;
        }
    }
    return Status::uncertified;
}


} // namespace

bool AMIPS_energy_p3_certified(const std::array<double, 12>& T, double& result)
{
    const Status s = amips_p3(T, result);
    return s == Status::certified || s == Status::zero_det;
}

} // namespace wmtk
