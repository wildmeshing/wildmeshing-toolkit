#pragma once

#include <predicates.h>
#include <wmtk/function/utils/autodiff.h>
#include <Eigen/Core>
#include <Eigen/Dense>
namespace wmtk::utils {

bool point_inside_triangle_2d_check(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    const Eigen::Vector2d& p)
{
    auto& nca = const_cast<Eigen::Vector2d&>(a);
    auto& ncb = const_cast<Eigen::Vector2d&>(b);
    auto& ncc = const_cast<Eigen::Vector2d&>(c);
    auto& ncp = const_cast<Eigen::Vector2d&>(p);

    auto tri1_orientation = orient2d(nca.data(), ncp.data(), ncc.data()) > 0;
    auto tri2_orientation = orient2d(nca.data(), ncb.data(), ncp.data()) > 0;
    auto tri3_orientation = orient2d(ncp.data(), ncb.data(), ncc.data()) > 0;

    return (tri1_orientation == tri2_orientation) && (tri2_orientation == tri3_orientation) &&
           (tri1_orientation == tri3_orientation);
}
} // namespace wmtk::utils