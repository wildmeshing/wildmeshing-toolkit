#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::utils {

// template <typename T>
// int wmtk_orient3d(
//     const Eigen::Ref<const Eigen::Vector3<T>>& p0,
//     const Eigen::Ref<const Eigen::Vector3<T>>& p1,
//     const Eigen::Ref<const Eigen::Vector3<T>>& p2,
//     const Eigen::Ref<const Eigen::Vector3<T>>& p3)
//{
//     throw std::runtime_error("unsupported type");
// }

int wmtk_orient3d(
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p0,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p1,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p2,
    const Eigen::Ref<const Eigen::Vector3<Rational>>& p3);

int wmtk_orient3d(
    const Eigen::Ref<const Eigen::Vector3<double>>& p0,
    const Eigen::Ref<const Eigen::Vector3<double>>& p1,
    const Eigen::Ref<const Eigen::Vector3<double>>& p2,
    const Eigen::Ref<const Eigen::Vector3<double>>& p3);


// assumes inputs are column vectors
template <typename Derived>
int wmtk_orient3d(const Eigen::MatrixBase<Derived>& A)
{
    assert(A.rows() == 3);
    assert(A.cols() == 4);
    const auto p0 = A.col(0);
    const auto p1 = A.col(1);
    const auto p2 = A.col(2);
    const auto p3 = A.col(3);
    return wmtk_orient3d(p0, p1, p2, p3);
}


// int wmtk_orient2d(
//     Eigen::Ref<const Eigen::Vector2<T>> p0,
//     Eigen::Ref<const Eigen::Vector2<T>> p1,
//     Eigen::Ref<const Eigen::Vector2<T>> p2)
//{
//     throw std::runtime_error("unsupported type");
// }

int wmtk_orient2d(double p0x, double p0y, double p1x, double p1y, double p2x, double p2y);

int wmtk_orient2d(
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p0,
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p1,
    const Eigen::Ref<const Eigen::Vector2<Rational>>& p2);

int wmtk_orient2d(
    const Eigen::Ref<const Eigen::Vector2<double>>& p0,
    const Eigen::Ref<const Eigen::Vector2<double>>& p1,
    const Eigen::Ref<const Eigen::Vector2<double>>& p2);

int wmtk_orient1d(const Rational& p0, const Rational& p1);

int wmtk_orient1d(double p0, double p1);

} // namespace wmtk::utils
