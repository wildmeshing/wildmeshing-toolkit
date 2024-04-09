#include "orient.hpp"
#include "predicates.h"

namespace wmtk::utils {
namespace {
bool is_rounded(const Eigen::Vector3<Rational>& p)
{
    return p[0].is_rounded() && p[1].is_rounded() && p[2].is_rounded();
}
bool is_rounded(const Eigen::Vector2<Rational>& p)
{
    return p[0].is_rounded() && p[1].is_rounded();
}

Rational determinant(const Eigen::Matrix<Rational, Eigen::Dynamic, Eigen::Dynamic, 0, 3, 3>& mat)
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
    return Rational();
}

} // namespace
template <>
int wmtk_orient3d(
    const Eigen::Vector3<Rational>& p0,
    const Eigen::Vector3<Rational>& p1,
    const Eigen::Vector3<Rational>& p2,
    const Eigen::Vector3<Rational>& p3)
{
    if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2) && is_rounded(p3)) {
        return wmtk_orient3d<double>(
            p0.cast<double>(),
            p1.cast<double>(),
            p2.cast<double>(),
            p3.cast<double>());
    } else {
        Eigen::Matrix3<Rational> M;
        M.row(0) = p1 - p0;
        M.row(1) = p2 - p0;
        M.row(2) = p3 - p0;
        const auto det = determinant(M);
        return det.get_sign();
    }
}

template <>
int wmtk_orient3d(
    const Eigen::Vector3<double>& p0,
    const Eigen::Vector3<double>& p1,
    const Eigen::Vector3<double>& p2,
    const Eigen::Vector3<double>& p3)
{
    Eigen::Vector3d p0nc = p0;
    Eigen::Vector3d p1nc = p1;
    Eigen::Vector3d p2nc = p2;
    Eigen::Vector3d p3nc = p3;

    return orient3d(p3nc.data(), p0nc.data(), p1nc.data(), p2nc.data());
}

template <>
int wmtk_orient2d(
    const Eigen::Vector2<Rational>& p0,
    const Eigen::Vector2<Rational>& p1,
    const Eigen::Vector2<Rational>& p2)
{
    if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2)) {
        return wmtk_orient2d<double>(p0.cast<double>(), p1.cast<double>(), p2.cast<double>());
    } else {
        Eigen::Matrix2<Rational> M;
        M.row(0) = p1 - p0;
        M.row(1) = p2 - p0;
        const auto det = determinant(M);
        return det.get_sign();
    }
}

template <>
int wmtk_orient2d(
    const Eigen::Vector2<double>& p0,
    const Eigen::Vector2<double>& p1,
    const Eigen::Vector2<double>& p2)
{
    Eigen::Vector2d p0nc = p0;
    Eigen::Vector2d p1nc = p1;
    Eigen::Vector2d p2nc = p2;

    return orient2d(p0nc.data(), p1nc.data(), p2nc.data());
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