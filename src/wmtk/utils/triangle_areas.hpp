#pragma once

#include <Eigen/Core>
#include <wmtk/Types.hpp>


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

// template get 2d tri area
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

// template get 3d tet area
template <typename ADerived, typename BDerived, typename CDerived, typename DDerived>
auto tetrahedron_signed_3d_area(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c,
    const Eigen::MatrixBase<DDerived>& d) -> typename ADerived::Scalar
{
    using Scalar = typename ADerived::Scalar;
    static_assert(ADerived::ColsAtCompileTime == 1);
    static_assert(BDerived::ColsAtCompileTime == 1);
    static_assert(CDerived::ColsAtCompileTime == 1);
    static_assert(DDerived::ColsAtCompileTime == 1);
    SquareMatrix<Scalar, 3> A;
    A.col(0) = a - d;
    A.col(1) = b - d;
    A.col(2) = c - d;
    return A.determinant() / Scalar(3);
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
