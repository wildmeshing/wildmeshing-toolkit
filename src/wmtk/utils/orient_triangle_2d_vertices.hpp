#pragma once

#include <predicates.h>
#include <Eigen/Core>
#include <Eigen/Dense>
namespace wmtk::utils {

void orient_triangle_2d_vertices(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    Eigen::Vector2d& a_oriented,
    Eigen::Vector2d& b_oriented,
    Eigen::Vector2d& c_oriented)
{
    auto& nca = const_cast<Eigen::Vector2d&>(a);
    auto& ncb = const_cast<Eigen::Vector2d&>(b);
    auto& ncc = const_cast<Eigen::Vector2d&>(c);

    if (orient2d(nca.data(), ncb.data(), ncc.data()) < 0) {
        a_oriented = nca;
        b_oriented = ncc;
        c_oriented = ncb;
    } else {
        a_oriented = nca;
        b_oriented = ncb;
        c_oriented = ncc;
    }
}
} // namespace wmtk::utils