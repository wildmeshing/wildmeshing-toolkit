#include "orient.hpp"
#include "predicates.h"

// clang-format off
#include <VolumeRemesher/numerics.h>
// clang-format on
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>

namespace wmtk::utils {

vol_rem::interval_number rational_to_interval(const Rational& r)
{
    if (r.is_rounded())
        return vol_rem::interval_number(r.to_double());
    else {
        const double inf = std::numeric_limits<double>::max();
        const double d = r.to_double();

        if (r < 0) return vol_rem::interval_number(-std::nextafter(d, -inf), d);
        if (r > 0) return vol_rem::interval_number(-d, std::nextafter(d, inf));
        return vol_rem::interval_number(0);
    }
}

void exactinit()
{
    // Thread-safe initialization using Meyers' singleton
    class MySingleton
    {
    public:
        static MySingleton& instance()
        {
            static MySingleton instance;
            return instance;
        }

    private:
        MySingleton() { ::exactinit(); }
    };
    MySingleton::instance();
}

namespace {
bool is_rounded(const Eigen::Ref<const Eigen::Vector3<Rational>>& p)
{
    return p[0].is_rounded() && p[1].is_rounded() && p[2].is_rounded();
}
bool is_rounded(const Eigen::Ref<const Eigen::Vector2<Rational>>& p)
{
    return p[0].is_rounded() && p[1].is_rounded();
}

template <typename T>
T determinant(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 3, 3>& mat)
{
    assert(mat.rows() == mat.cols());

    if (mat.rows() == 1)
        return mat(0);
    else if (mat.rows() == 2)
        return mat(0, 0) * mat(1, 1) - mat(0, 1) * mat(1, 0);
    else if (mat.rows() == 3)
        return mat(0, 0) * (mat(1, 1) * mat(2, 2) - mat(1, 2) * mat(2, 1)) -
               mat(0, 1) * (mat(1, 0) * mat(2, 2) - mat(1, 2) * mat(2, 0)) +
               mat(0, 2) * (mat(1, 0) * mat(2, 1) - mat(1, 1) * mat(2, 0));

    assert(false);
    return T();
}

} // namespace
int wmtk_orient3d(
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p0,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p1,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p2,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p3)
{
    if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2) && is_rounded(p3)) {
        return wmtk_orient3d(
            p0.cast<double>(),
            p1.cast<double>(),
            p2.cast<double>(),
            p3.cast<double>());
    } else {
        // Super Fast version using double
        // Eigen::Vector3<double> p0r_d;
        // Eigen::Vector3<double> p1r_d;
        // Eigen::Vector3<double> p2r_d;
        // Eigen::Vector3<double> p3r_d;

        // for (int64_t i = 0; i < 3; ++i) {
        //     p0r_d[i] = p0[i].to_double();
        //     p1r_d[i] = p1[i].to_double();
        //     p2r_d[i] = p2[i].to_double();
        //     p3r_d[i] = p3[i].to_double();
        // }

        // Eigen::Matrix3<double> M_d;
        // M_d.row(0) = p0r_d - p3r_d;
        // M_d.row(1) = p1r_d - p3r_d;
        // M_d.row(2) = p2r_d - p3r_d;

        // const auto det_d = determinant<double>(M_d);

        // Fast version using intervals
        Eigen::Vector3<vol_rem::interval_number> p0r_i;
        Eigen::Vector3<vol_rem::interval_number> p1r_i;
        Eigen::Vector3<vol_rem::interval_number> p2r_i;
        Eigen::Vector3<vol_rem::interval_number> p3r_i;

        for (int64_t i = 0; i < 3; ++i) {
            p0r_i[i] = rational_to_interval(p0[i]);
            p1r_i[i] = rational_to_interval(p1[i]);
            p2r_i[i] = rational_to_interval(p2[i]);
            p3r_i[i] = rational_to_interval(p3[i]);
        }

        Eigen::Matrix3<vol_rem::interval_number> M_i;
        M_i.row(0) = p0r_i - p3r_i;
        M_i.row(1) = p1r_i - p3r_i;
        M_i.row(2) = p2r_i - p3r_i;


        const auto det_i = determinant<vol_rem::interval_number>(M_i);
        // assert(!det.is_rounded());
        if (det_i.signIsReliable()) {
            return det_i.sign();
        }

        // Slow version using rationals
        Eigen::Vector3<Rational> p0r;
        Eigen::Vector3<Rational> p1r;
        Eigen::Vector3<Rational> p2r;
        Eigen::Vector3<Rational> p3r;

        for (int64_t i = 0; i < 3; ++i) {
            p0r[i] = Rational(p0[i], false);
            p1r[i] = Rational(p1[i], false);
            p2r[i] = Rational(p2[i], false);
            p3r[i] = Rational(p3[i], false);
        }

        Eigen::Matrix3<Rational> M;
        M.row(0) = p0r - p3r;
        M.row(1) = p1r - p3r;
        M.row(2) = p2r - p3r;

#ifndef NDEBUG
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                assert(!M(i, j).is_rounded());
            }
        }
#endif

        const auto det = determinant<Rational>(M);
        assert(!det.is_rounded());

        return det.get_sign();
    }
}

int wmtk_orient3d(
    const Eigen::Ref<const Eigen::Vector3<double>>& p0,
    const Eigen::Ref<const Eigen::Vector3<double>>& p1,
    const Eigen::Ref<const Eigen::Vector3<double>>& p2,
    const Eigen::Ref<const Eigen::Vector3<double>>& p3)
{
    Eigen::Vector3d p0nc = p0;
    Eigen::Vector3d p1nc = p1;
    Eigen::Vector3d p2nc = p2;
    Eigen::Vector3d p3nc = p3;

    exactinit();
    const auto res = orient3d(p0nc.data(), p1nc.data(), p2nc.data(), p3nc.data());

    if (res > 0)
        return 1;
    else if (res < 0)
        return -1;
    else
        return 0;
}

int wmtk_orient2d(
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p0,
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p1,
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p2)
{
    if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2)) {
        return wmtk_orient2d(p0.cast<double>(), p1.cast<double>(), p2.cast<double>());
    } else {
        Eigen::Vector2<Rational> p0r;
        Eigen::Vector2<Rational> p1r;
        Eigen::Vector2<Rational> p2r;

        for (int64_t i = 0; i < 2; ++i) {
            p0r[i] = Rational(p0[i], false);
            p1r[i] = Rational(p1[i], false);
            p2r[i] = Rational(p2[i], false);
        }

        Eigen::Matrix2<Rational> M;
        M.row(0) = p1r - p0r;
        M.row(1) = p2r - p0r;
        const auto det = determinant<Rational>(M);
        return det.get_sign();
    }
}

int wmtk_orient2d(
    const Eigen::Ref<const Eigen::Vector2<double>>& p0,
    const Eigen::Ref<const Eigen::Vector2<double>>& p1,
    const Eigen::Ref<const Eigen::Vector2<double>>& p2)
{
    Eigen::Vector2d p0nc = p0;
    Eigen::Vector2d p1nc = p1;
    Eigen::Vector2d p2nc = p2;

    exactinit();
    const auto res = orient2d(p0nc.data(), p1nc.data(), p2nc.data());

    if (res > 0)
        return 1;
    else if (res < 0)
        return -1;
    else
        return 0;
}

int wmtk_orient1d(const Rational& p0, const Rational& p1)
{
    return p0 == p1 ? 0 : (p0 < p1 ? -1 : 1);
}

int wmtk_orient1d(double p0, double p1)
{
    return p0 == p1 ? 0 : (p0 < p1 ? -1 : 1);
}

} // namespace wmtk::utils
