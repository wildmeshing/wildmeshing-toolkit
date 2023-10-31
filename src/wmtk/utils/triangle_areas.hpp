#pragma once

#include <Eigen/Core>
namespace wmtk {
class Tuple;
class TriMesh;
namespace attribute {
template <typename T>
class MeshAttributeHandle;
}
} // namespace wmtk
namespace wmtk::utils {

// template get 3d tri area
template <typename ADerived, typename BDerived, typename CDerived>
auto triangle_3d_area(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
{
    auto ba = b - a;
    auto ca = c - a;
    return typename ADerived::Scalar(.5) * ba.cross(ca).norm();
}

// template get 3d tri area
template <typename ADerived, typename BDerived, typename CDerived>
auto triangle_signed_2d_area(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
{
    auto ba = (b - a).eval();
    auto ca = (c - a).eval();
    return typename ADerived::Scalar(.5) * ba.homogeneous().cross(ca.homogeneous()).z();
}

template <typename ADerived, typename BDerived, typename CDerived>
auto triangle_unsigned_2d_area(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
{
    return std::abs(triangle_signed_2d_area(a, b, c));
}

template <typename ADerived, typename BDerived, typename CDerived>
bool triangle_2d_orientation(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c)
{
    auto res = igl::predicates::orient2d(a, b, c);
    if (res == igl::predicates::Orientation::POSITIVE)
        return true;
    else
        return false;
}

} // namespace wmtk::utils
