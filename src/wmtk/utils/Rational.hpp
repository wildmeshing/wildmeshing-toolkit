#pragma once

#include <gmp.h>
#include <iostream>

namespace wmtk {

class Rational
{
public:
    mpq_t value;
    void canonicalize() { mpq_canonicalize(value); }
    int get_sign() { return mpq_sgn(value); }

    Rational()
    {
        mpq_init(value);
        mpq_set_d(value, 0);
    }

    Rational(double d)
    {
        mpq_init(value);
        mpq_set_d(value, d);
        // canonicalize();
    }

    Rational(const mpq_t& v_)
    {
        mpq_init(value);
        mpq_set(value, v_);
        // canonicalize();
    }

    Rational(const Rational& other)
    {
        mpq_init(value);
        mpq_set(value, other.value);
    }

    Rational(const std::string& num, const std::string& denom)
    {
        mpq_init(value);
        std::string tmp = num + "/" + denom;
        mpq_set_str(value, tmp.c_str(), 10);
    }

    std::string numerator() const
    {
        mpz_t num;
        mpz_init(num);

        mpq_get_num(num, value);
        std::string v(mpz_get_str(NULL, 10, num));

        mpz_clear(num);
        return v;
    }

    std::string denominator() const
    {
        mpz_t denom;
        mpz_init(denom);
        mpq_get_den(denom, value);

        std::string v(mpz_get_str(NULL, 10, denom));

        mpz_clear(denom);
        return v;
    }


    ~Rational() { mpq_clear(value); }

    friend Rational operator+(const Rational& x, const Rational& y)
    {
        Rational r_out;
        mpq_add(r_out.value, x.value, y.value);
        return r_out;
    }

    friend Rational operator-(const Rational& x, const Rational& y)
    {
        Rational r_out;
        mpq_sub(r_out.value, x.value, y.value);
        return r_out;
    }


    friend Rational operator-(const Rational& x)
    {
        Rational r_out;
        mpq_neg(r_out.value, x.value);
        return r_out;
    }

    friend Rational pow(const Rational& x, int p)
    {
        Rational r_out = x;
        for (int i = 1; i < std::abs(p); i++) {
            r_out = r_out * x;
        }
        if (p < 0) return 1 / r_out;
        return r_out;
    }

    friend Rational operator*(const Rational& x, const Rational& y)
    {
        Rational r_out;
        mpq_mul(r_out.value, x.value, y.value);
        return r_out;
    }

    friend Rational operator/(const Rational& x, const Rational& y)
    {
        Rational r_out;
        mpq_div(r_out.value, x.value, y.value);
        return r_out;
    }

    Rational& operator=(const Rational& x)
    {
        if (this == &x) return *this;
        mpq_set(value, x.value);
        return *this;
    }

    Rational& operator=(const double x)
    {
        mpq_set_d(value, x);
        //            canonicalize();
        return *this;
    }

    //> < ==
    friend bool operator<(const Rational& r, const Rational& r1)
    {
        return mpq_cmp(r.value, r1.value) < 0;
    }

    friend bool operator>(const Rational& r, const Rational& r1)
    {
        return mpq_cmp(r.value, r1.value) > 0;
    }

    friend bool operator<=(const Rational& r, const Rational& r1)
    {
        return mpq_cmp(r.value, r1.value) <= 0;
    }

    friend bool operator>=(const Rational& r, const Rational& r1)
    {
        return mpq_cmp(r.value, r1.value) >= 0;
    }

    friend bool operator==(const Rational& r, const Rational& r1)
    {
        return mpq_equal(r.value, r1.value);
    }

    friend bool operator!=(const Rational& r, const Rational& r1)
    {
        return !mpq_equal(r.value, r1.value);
    }

    // to double
    double to_double() const { return mpq_get_d(value); }
    explicit operator double() const { return to_double(); }

    friend Rational abs(const Rational& r0)
    {
        Rational r;
        mpq_abs(r.value, r0.value);
        return r;
    }

    //<<
    friend std::ostream& operator<<(std::ostream& os, const Rational& r)
    {
        os << mpq_get_d(r.value);
        return os;
    }

    template <typename T>
    void init(const T& v)
    {
        mpq_set(value, v);
    }
};


} // namespace wmtk
