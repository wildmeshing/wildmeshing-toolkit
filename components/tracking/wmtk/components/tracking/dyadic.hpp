#pragma once

#include <gmp.h>

#include <cmath>
#include <utility>

namespace wmtk::components::tracking::exact {

/**
 * An exact dyadic number m 2^e, m an integer (GMP). Every double is one, and so is every sum,
 * difference and product of them: the exact values of the map's computations, which divide by
 * nothing (homogeneous coordinates are scaled by powers of two instead), are all dyadic. Unlike a
 * rational, a dyadic number never needs a gcd: m is kept odd (or zero).
 */
class Dyadic
{
public:
    Dyadic() { mpz_init(m_m); }
    Dyadic(const double d)
    {
        mpz_init(m_m);
        set(d);
    }
    Dyadic(const Dyadic& o)
        : m_e(o.m_e)
    {
        mpz_init_set(m_m, o.m_m);
    }
    Dyadic(Dyadic&& o) noexcept
        : m_e(o.m_e)
    {
        mpz_init(m_m);
        mpz_swap(m_m, o.m_m);
    }
    Dyadic& operator=(const Dyadic& o)
    {
        if (this != &o) {
            mpz_set(m_m, o.m_m);
            m_e = o.m_e;
        }
        return *this;
    }
    Dyadic& operator=(Dyadic&& o) noexcept
    {
        mpz_swap(m_m, o.m_m);
        std::swap(m_e, o.m_e);
        return *this;
    }
    ~Dyadic() { mpz_clear(m_m); }

    int sign() const { return mpz_sgn(m_m); }

    /// The value rounded towards zero to a double (to 0 only below the smallest normal double).
    double to_double() const
    {
        if (sign() == 0) return 0;
        const long bits = long(mpz_sizeinbase(m_m, 2));
        if (bits <= 53) return std::ldexp(mpz_get_d(m_m), int(m_e));
        mpz_t q;
        mpz_init(q);
        mpz_tdiv_q_2exp(q, m_m, mp_bitcnt_t(bits - 53));
        const double d = std::ldexp(mpz_get_d(q), int(m_e + bits - 53));
        mpz_clear(q);
        return d;
    }

    /// The exponent of the leading bit: 2^ilogb <= |x| < 2^(ilogb + 1). x must not be 0.
    long ilogb() const { return long(mpz_sizeinbase(m_m, 2)) - 1 + m_e; }

    /// x 2^k, exactly.
    Dyadic scaled(const long k) const
    {
        Dyadic r(*this);
        if (r.sign() != 0) r.m_e += k;
        return r;
    }

    friend Dyadic operator*(const Dyadic& a, const Dyadic& b)
    {
        Dyadic r;
        if (a.sign() == 0 || b.sign() == 0) return r;
        mpz_mul(r.m_m, a.m_m, b.m_m);
        r.m_e = a.m_e + b.m_e;
        return r;
    }
    friend Dyadic operator+(const Dyadic& a, const Dyadic& b) { return sum(a, b, false); }
    friend Dyadic operator-(const Dyadic& a, const Dyadic& b) { return sum(a, b, true); }
    friend Dyadic operator-(const Dyadic& a)
    {
        Dyadic r(a);
        mpz_neg(r.m_m, r.m_m);
        return r;
    }
    friend bool operator==(const Dyadic& a, const Dyadic& b)
    {
        return a.m_e == b.m_e && mpz_cmp(a.m_m, b.m_m) == 0;
    }
    friend bool operator!=(const Dyadic& a, const Dyadic& b) { return !(a == b); }

private:
    mpz_t m_m;
    long m_e = 0;

    void set(const double d)
    {
        if (d == 0 || !std::isfinite(d)) {
            mpz_set_ui(m_m, 0);
            m_e = 0;
            return;
        }
        int ex = 0;
        const double f = std::frexp(d, &ex); // d = f 2^ex, 1/2 <= |f| < 1
        // f 2^53 is an integer below 2^53, so a double holds it and mpz_set_d takes it exactly
        // (mpz_set_si would not: long is 32 bits on Windows).
        mpz_set_d(m_m, std::ldexp(f, 53));
        m_e = long(ex) - 53;
        canonical(); // odd mantissa
    }

    /// m odd (or zero, with e = 0): one representation per value.
    void canonical()
    {
        if (mpz_sgn(m_m) == 0) {
            m_e = 0;
            return;
        }
        const mp_bitcnt_t z = mpz_scan1(m_m, 0);
        if (z > 0) {
            mpz_tdiv_q_2exp(m_m, m_m, z);
            m_e += long(z);
        }
    }

    static Dyadic sum(const Dyadic& a, const Dyadic& b, const bool minus)
    {
        if (b.sign() == 0) return a;
        if (a.sign() == 0) return minus ? -b : b;
        Dyadic r;
        if (a.m_e <= b.m_e) {
            mpz_mul_2exp(r.m_m, b.m_m, mp_bitcnt_t(b.m_e - a.m_e));
            if (minus) {
                mpz_sub(r.m_m, a.m_m, r.m_m);
            } else {
                mpz_add(r.m_m, a.m_m, r.m_m);
            }
            r.m_e = a.m_e;
        } else {
            mpz_mul_2exp(r.m_m, a.m_m, mp_bitcnt_t(a.m_e - b.m_e));
            if (minus) {
                mpz_sub(r.m_m, r.m_m, b.m_m);
            } else {
                mpz_add(r.m_m, r.m_m, b.m_m);
            }
            r.m_e = b.m_e;
        }
        r.canonical();
        return r;
    }
};

} // namespace wmtk::components::tracking::exact
