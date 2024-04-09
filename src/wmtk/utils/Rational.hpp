#pragma once

#include <gmp.h>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <string>

namespace wmtk {

class Rational
{
public:
    void canonicalize()
    {
        if (is_rounded) return;
        mpq_canonicalize(value);
    }

    int get_sign()
    {
        if (is_rounded) return d_value == 0 ? 0 : (d_value < 0 ? -1 : 1);

        return mpq_sgn(value);
    }

    Rational(bool rounded = false)
        : Rational(0.0, rounded)
    {}

    Rational(int v, bool rounded = false)
        : Rational((double)v, rounded)
    {}

    Rational(double d, bool rounded = false)
        : is_rounded(rounded)
    {
        if (is_rounded) {
            d_value = d;
        } else {
            mpq_init(value);
            mpq_set_d(value, d);

            d_value = std::numeric_limits<double>::lowest();
            // canonicalize();
        }
    }

    Rational(const mpq_t& v_)
    {
        mpq_init(value);
        mpq_set(value, v_);
        // canonicalize();
        is_rounded = false;

        d_value = std::numeric_limits<double>::lowest();
    }

    Rational(const Rational& other)
        : d_value(other.d_value)
        , is_rounded(other.is_rounded)
    {
        if (!is_rounded) {
            mpq_init(value);
            mpq_set(value, other.value);
        }
    }

    ~Rational()
    {
        if (!is_rounded) mpq_clear(value);
    }

    friend Rational operator+(const Rational& x, const Rational& y)
    {
        if (x.is_rounded && y.is_rounded) {
            return Rational(x.d_value + y.d_value, true);
        }

        Rational r_out;

        if (x.is_rounded && !y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, x.d_value);
            mpq_add(r_out.value, tmp, y.value);
            mpq_clear(tmp);
        } else if (!x.is_rounded && y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, y.d_value);
            mpq_add(r_out.value, x.value, tmp);
            mpq_clear(tmp);
        } else
            mpq_add(r_out.value, x.value, y.value);

        return r_out;
    }

    friend Rational operator-(const Rational& x, const Rational& y)
    {
        if (x.is_rounded && y.is_rounded) {
            return Rational(x.d_value - y.d_value, true);
        }

        Rational r_out;

        if (x.is_rounded && !y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, x.d_value);
            mpq_sub(r_out.value, tmp, y.value);
            mpq_clear(tmp);
        } else if (!x.is_rounded && y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, y.d_value);
            mpq_sub(r_out.value, x.value, tmp);
            mpq_clear(tmp);
        } else
            mpq_sub(r_out.value, x.value, y.value);

        return r_out;
    }


    friend Rational operator-(const Rational& x)
    {
        if (x.is_rounded) return Rational(-x.d_value, true);

        Rational r_out;
        mpq_neg(r_out.value, x.value);
        return r_out;
    }

    friend Rational pow(const Rational& x, int p)
    {
        if (x.is_rounded) return Rational(std::pow(x.d_value, p), true);

        Rational r_out = x;
        for (int i = 1; i < std::abs(p); i++) {
            r_out = r_out * x;
        }
        if (p < 0) return Rational(1.0, false) / r_out;
        return r_out;
    }

    friend Rational operator*(const Rational& x, const Rational& y)
    {
        if (x.is_rounded && y.is_rounded) {
            return Rational(x.d_value * y.d_value, true);
        }

        Rational r_out;

        if (x.is_rounded && !y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, x.d_value);
            mpq_mul(r_out.value, tmp, y.value);
            mpq_clear(tmp);
        } else if (!x.is_rounded && y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, y.d_value);
            mpq_mul(r_out.value, x.value, tmp);
            mpq_clear(tmp);
        } else
            mpq_mul(r_out.value, x.value, y.value);

        return r_out;
    }

    friend Rational operator/(const Rational& x, const Rational& y)
    {
        if (x.is_rounded && y.is_rounded) {
            return Rational(x.d_value / y.d_value, true);
        }

        Rational r_out;

        if (x.is_rounded && !y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, x.d_value);
            mpq_div(r_out.value, tmp, y.value);
            mpq_clear(tmp);
        } else if (!x.is_rounded && y.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, y.d_value);
            mpq_div(r_out.value, x.value, tmp);
            mpq_clear(tmp);
        } else
            mpq_div(r_out.value, x.value, y.value);

        return r_out;
    }

    Rational& operator=(const Rational& x)
    {
        if (this == &x) return *this;

        if (is_rounded && x.is_rounded)
            d_value = x.d_value;
        else if (!is_rounded && x.is_rounded)
            mpq_set_d(value, x.d_value);
        else if (is_rounded && !x.is_rounded) {
            is_rounded = false;
            mpq_init(value);
            mpq_set(value, x.value);
        } else
            mpq_set(value, x.value);
        return *this;
    }

    Rational& operator=(const double x)
    {
        if (is_rounded)
            d_value = x;
        else {
            mpq_set_d(value, x);
            d_value = std::numeric_limits<double>::lowest();
        }
        //            canonicalize();
        return *this;
    }

    inline friend int cmp(const Rational& r, const Rational& r1)
    {
        if (r.is_rounded && r1.is_rounded)
            return r.d_value == r1.d_value ? 0 : (r.d_value > r1.d_value ? 1 : -1);

        if (r.is_rounded && !r1.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, r.d_value);
            int res = mpq_cmp(tmp, r1.value);
            mpq_clear(tmp);
            return res;
        }

        if (!r.is_rounded && r1.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, r1.d_value);
            int res = mpq_cmp(r.value, tmp);
            mpq_clear(tmp);
            return res;
        }

        return mpq_cmp(r.value, r1.value);
    }

    //> < ==
    friend bool operator<(const Rational& r, const Rational& r1) { return cmp(r, r1) < 0; }
    friend bool operator>(const Rational& r, const Rational& r1) { return cmp(r, r1) > 0; }
    friend bool operator<=(const Rational& r, const Rational& r1) { return cmp(r, r1) <= 0; }
    friend bool operator>=(const Rational& r, const Rational& r1) { return cmp(r, r1) >= 0; }

    friend bool operator==(const Rational& r, const Rational& r1)
    {
        if (r.is_rounded && r1.is_rounded) return r.d_value == r1.d_value;

        if (r.is_rounded && !r1.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, r.d_value);
            bool res = mpq_equal(tmp, r1.value);
            mpq_clear(tmp);
            return res;
        }

        if (!r.is_rounded && r1.is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, r1.d_value);
            bool res = mpq_equal(r.value, tmp);
            mpq_clear(tmp);
            return res;
        }

        return mpq_equal(r.value, r1.value);
    }

    friend bool operator!=(const Rational& r, const Rational& r1) { return !(r == r1); }

    // to double
    inline double to_double() const
    {
        if (is_rounded) return d_value;
        return mpq_get_d(value);
    }

    explicit operator double() const { return to_double(); }

    inline friend Rational abs(const Rational& r0)
    {
        if (r0.is_rounded) {
            return Rational(std::abs(r0.d_value), true);
        } else {
            Rational r;
            mpq_abs(r.value, r0.value);
            return r;
        }
    }

    //<<
    friend std::ostream& operator<<(std::ostream& os, const Rational& r)
    {
        os << r.to_double();
        return os;
    }

    template <typename T>
    void init(const T& v)
    {
        mpq_set(value, v);
        is_rounded = false;
    }

    inline void round()
    {
        if (is_rounded) return;

        is_rounded = true;
        d_value = this->to_double();
        mpq_clear(value);
    }

    void init_from_binary(const std::string& v) { mpq_set_str(value, v.c_str(), 2); }
    std::string to_binary() const
    {
        if (is_rounded) {
            mpq_t tmp;
            mpq_init(tmp);
            mpq_set_d(tmp, d_value);

            std::string v(mpq_get_str(NULL, 2, tmp));
            mpq_clear(tmp);
            return v;
        }

        std::string v(mpq_get_str(NULL, 2, value));
        return v;
    }

    inline std::string serialize() const
    {
        return numerator() + "/" + denominator() + "/" + (is_rounded ? "1" : "0");
    }

    Rational(const std::string& data)
    {
        std::regex regex{R"([/]+)"}; // split on /
        std::sregex_token_iterator it{data.begin(), data.end(), regex, -1};
        std::vector<std::string> tokens{it, {}};
        assert(tokens.size() == 3);

        const auto num = tokens[0];
        const auto denom = tokens[1];
        is_rounded = tokens[2][0] == '1';

        if (is_rounded) {
            mpq_t tmp_r;
            mpq_init(tmp_r);
            std::string tmp = num + "/" + denom;
            mpq_set_str(tmp_r, tmp.c_str(), 10);

            d_value = mpq_get_d(tmp_r);
            mpq_clear(tmp_r);

        } else {
            mpq_init(value);
            std::string tmp = num + "/" + denom;
            mpq_set_str(value, tmp.c_str(), 10);
            d_value = std::numeric_limits<double>::lowest();
        }
    }

private:
    mpq_t value;
    double d_value;
    bool is_rounded;

    std::string numerator() const
    {
        mpq_t tmp;
        mpq_init(tmp);
        if (is_rounded)
            mpq_set_d(tmp, d_value);
        else
            mpq_set(tmp, value);


        mpz_t num;
        mpz_init(num);

        mpq_get_num(num, tmp);
        std::string v(mpz_get_str(NULL, 10, num));

        mpz_clear(num);
        mpq_clear(tmp);

        return v;
    }

    std::string denominator() const
    {
        mpq_t tmp;
        mpq_init(tmp);
        if (is_rounded)
            mpq_set_d(tmp, d_value);
        else
            mpq_set(tmp, value);

        mpz_t denom;
        mpz_init(denom);
        mpq_get_den(denom, tmp);

        std::string v(mpz_get_str(NULL, 10, denom));

        mpz_clear(denom);
        mpq_clear(tmp);

        return v;
    }
};


} // namespace wmtk
