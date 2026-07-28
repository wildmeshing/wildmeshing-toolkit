#pragma once

#include <cmath>

// A cross-platform-deterministic atan2.
//
// std::atan2 / std::atan are not correctly rounded and differ by a few ULP
// between libm implementations (e.g. macOS libm vs glibc), which makes anything
// that feeds their result into a discrete decision diverge across OSes. This
// version uses only the IEEE-754 correctly-rounded operations (+ - * /) and a
// fixed polynomial with hardcoded constants, so it produces the same bits
// everywhere while staying within ~1 ULP of the true value.
//
// The atan core (argument-reduction breakpoints and polynomial coefficients) is
// adapted from fdlibm's s_atan.c (Sun Microsystems, freely distributable:
// "Developed at SunSoft ... Permission to use, copy, modify, and distribute this
// software is freely granted, provided that this notice is preserved.").

namespace wmtk::utils {

inline double deterministic_atan(double x)
{
    static const double atanhi[4] = {
        4.63647609000806093515e-01, // atan(0.5)hi
        7.85398163397448278999e-01, // atan(1.0)hi
        9.82793723247329054082e-01, // atan(1.5)hi
        1.57079632679489655800e+00 // atan(inf)hi
    };
    static const double atanlo[4] = {
        2.26987774529616870924e-17,
        3.06161699786838301793e-17,
        1.39033110312309984516e-17,
        6.12323399573676603587e-17};
    static const double aT[11] = {
        3.33333333333329318027e-01,
        -1.99999999998764832476e-01,
        1.42857142725034663711e-01,
        -1.11111104054623557880e-01,
        9.09088713343650656196e-02,
        -7.69187620504482999495e-02,
        6.66107313738753120669e-02,
        -5.83357013379057348645e-02,
        4.97687799461593236017e-02,
        -3.65315727442169155270e-02,
        1.62858201153657823623e-02};

    if (std::isnan(x)) return x;
    const bool neg = x < 0.0;
    double ax = neg ? -x : x;

    int id;
    if (ax < 7.0 / 16.0) {
        id = -1; // no reduction
    } else if (ax < 11.0 / 16.0) {
        id = 0;
        ax = (2.0 * ax - 1.0) / (2.0 + ax);
    } else if (ax < 19.0 / 16.0) {
        id = 1;
        ax = (ax - 1.0) / (1.0 + ax);
    } else if (ax < 39.0 / 16.0) {
        id = 2;
        ax = (ax - 1.5) / (1.0 + 1.5 * ax);
    } else {
        id = 3;
        ax = -1.0 / ax;
    }

    const double z = ax * ax;
    const double w = z * z;
    const double s1 =
        z * (aT[0] + w * (aT[2] + w * (aT[4] + w * (aT[6] + w * (aT[8] + w * aT[10])))));
    const double s2 = w * (aT[1] + w * (aT[3] + w * (aT[5] + w * (aT[7] + w * aT[9]))));

    double r;
    if (id < 0) {
        r = ax - ax * (s1 + s2);
    } else {
        r = atanhi[id] - ((ax * (s1 + s2) - atanlo[id]) - ax);
    }
    return neg ? -r : r;
}

inline double deterministic_atan2(double y, double x)
{
    // pi and pi/2 as their nearest-double literals (identical on every platform).
    static const double PI = 3.14159265358979311600;
    static const double PI_2 = 1.57079632679489655800;

    if (std::isnan(x) || std::isnan(y)) return x + y;
    if (x == 0.0) {
        if (y > 0.0) return PI_2;
        if (y < 0.0) return -PI_2;
        return 0.0; // atan2(0,0)
    }
    const double a = deterministic_atan(y / x);
    if (x > 0.0) return a; // quadrants I, IV
    return (y >= 0.0) ? a + PI : a - PI; // quadrants II, III
}

} // namespace wmtk::utils
