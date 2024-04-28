#pragma once

#include <Eigen/Core>

#include <gmp.h>
#include <iostream>
#include <string>


namespace wmtk {

class Rational
{
public:
    Rational(bool rounded = false);
    Rational(int v, bool rounded = false);
    Rational(double d, bool rounded = false);
    Rational(const mpq_t& v_);
    Rational(const Rational& other);
    Rational(const Rational& other, bool rounded);
    Rational(const Eigen::VectorX<char>& data);
    Rational(const std::string& data, bool rounded = false);

    Rational& operator=(const Rational& x);
    Rational& operator=(const double x);

    template <typename T>
    void init(const T& v)
    {
        mpq_set(value, v);
        m_is_rounded = false;
    }

    ~Rational();

    void canonicalize();

    friend Rational operator+(const Rational& x, const Rational& y);
    friend Rational operator-(const Rational& x, const Rational& y);

    friend Rational operator-(const Rational& x);

    friend Rational pow(const Rational& x, int p);
    friend Rational abs(const Rational& r0);
    int get_sign() const;

    friend Rational operator*(const Rational& x, const Rational& y);
    friend Rational operator/(const Rational& x, const Rational& y);

    //> < ==
    friend bool operator<(const Rational& r, const Rational& r1) { return cmp(r, r1) < 0; }
    friend bool operator>(const Rational& r, const Rational& r1) { return cmp(r, r1) > 0; }
    friend bool operator<=(const Rational& r, const Rational& r1) { return cmp(r, r1) <= 0; }
    friend bool operator>=(const Rational& r, const Rational& r1) { return cmp(r, r1) >= 0; }

    friend bool operator==(const Rational& r, const Rational& r1);
    friend bool operator!=(const Rational& r, const Rational& r1);

    // to double
    double to_double() const;
    explicit operator double() const;

    friend std::ostream& operator<<(std::ostream& os, const Rational& r);

    inline void round()
    {
        if (m_is_rounded) return;

        d_value = this->to_double();
        m_is_rounded = true;
        mpq_clear(value);
    }

    void init_from_binary(const std::string& v);
    std::string to_binary() const;

    std::string serialize() const;

    inline bool is_rounded() const { return m_is_rounded; }


private:
    mpq_t value;
    double d_value;
    bool m_is_rounded;

    friend int cmp(const Rational& r, const Rational& r1);

    std::string numerator() const;
    std::string denominator() const;
};


} // namespace wmtk
