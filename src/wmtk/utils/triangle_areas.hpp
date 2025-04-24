#pragma once

#include <Eigen/Dense>


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
    return std::abs(triangle_signed_2d(a, b, c));
}

// private:
//     class Internal
//     {
//     private:
//         Internal();

//     public:
//         Internal& instance();
//         double orient2d_aux(double pa[2], double pb[2], double pc[2]);
//         double orient3d_aux(double pa[3], double pb[3], double pc[3], double pd[3]);
//         double incircle_aux(double pa[2], double pb[2], double pc[2], double pd[2]);
//         double insphere_aux(double pa[3], double pb[3], double pc[3], double pd[3], double
//         pe[3]);
//     };
// };
} // namespace wmtk::utils
