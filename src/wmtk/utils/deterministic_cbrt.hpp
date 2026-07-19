#pragma once

#include <cmath>

namespace wmtk::utils {

/**
 * @brief A cube root that is bit-identical on every IEEE-754 platform.
 *
 * std::cbrt and std::pow are not correctly rounded and differ by ~1 ULP between
 * libm implementations (e.g. macOS libm vs glibc), which makes any iterative
 * geometry code that feeds their result into discrete decisions diverge across
 * OSes/architectures. This routine avoids that: it range-reduces with frexp/ldexp
 * (both exact) and then runs a fixed number of Newton iterations using only the
 * IEEE-754 correctly-rounded operations + - * / (and a hardcoded constant for
 * 2^(1/3), 2^(2/3)). The exact same sequence of correctly-rounded operations runs
 * everywhere, so the result is identical everywhere.
 *
 * The result is not guaranteed to be the correctly-rounded cube root (it is within
 * a couple of ULP of it), but it is deterministic, which is what reproducible
 * meshing output requires.
 */
inline double deterministic_cbrt(double x)
{
    if (x == 0.0 || std::isnan(x) || std::isinf(x)) return x;

    const bool neg = x < 0.0;
    const double a = neg ? -x : x;

    // a = m * 2^e, with m in [0.5, 1). frexp/ldexp only touch the exponent, so
    // they introduce no rounding.
    int e = 0;
    const double m = std::frexp(a, &e);

    // Split e = 3*q + r with r in {0, 1, 2} so the 2^q factor is an exact scaling.
    int q = e / 3;
    int r = e - 3 * q;
    if (r < 0) {
        r += 3;
        q -= 1;
    }

    // Initial guess for cbrt(m), m in [0.5, 1): quadratic through
    // (0.5, 2^-1/3), (0.75, 0.75^1/3), (1, 1). Accurate to <1%, so a few Newton
    // steps reach full double precision.
    double t = 0.493724 + m * (0.693628 - m * 0.187352);

    // Newton for t^3 = m: t <- (2 t + m / t^2) / 3. Quadratic convergence; 5 steps
    // reach the fixed point from the guess above for every m in [0.5, 1).
    for (int i = 0; i < 5; ++i) t = (2.0 * t + m / (t * t)) / 3.0;

    // cbrt(a) = cbrt(m) * cbrt(2^r) * 2^q. cbrt(2^r) is a compile-time-rounded
    // literal (identical on every platform); the final ldexp is exact.
    static const double cbrt2r[3] = {
        1.0,
        1.2599210498948731648, // 2^(1/3)
        1.5874010519681993614 // 2^(2/3)
    };
    const double res = std::ldexp(t * cbrt2r[r], q);
    return neg ? -res : res;
}

} // namespace wmtk::utils
