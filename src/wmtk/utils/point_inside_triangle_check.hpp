#pragma once

#include <predicates.h>
#include <wmtk/function/utils/autodiff.h>
#include <Eigen/Core>

// template get 3d tri area
template <typename ADerived, typename BDerived, typename CDerived, typename DDerived>
bool point_inside_triangle_2d_check(
    const Eigen::MatrixBase<ADerived>& a,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<CDerived>& c,
    const Eigen::MatrixBase<DDerived>& p)
{
    auto tri1_orientation = orient2d(a.data(), p.data(), c.data());
    auto tri2_orientation = orient2d(a.data(), b.data(), p.data());
    auto tri3_orientation = orient2d(p.data(), b.data(), c.data());
    return (tri1_orientation == tri2_orientation) && (tri2_orientation == tri3_orientation) &&
           (tri1_orientation == tri3_orientation);
}
