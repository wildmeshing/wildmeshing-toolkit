#pragma once

#include <Eigen/Core>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/Types.hpp>

namespace wmtk::components::c1_simplification {

bool point_in_tri(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c);
Eigen::Vector2d barycentric_coord_in_tri(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c);

/////////////////////////////
//// clough tocher patch ////
/////////////////////////////

int triangle_ind(const double& u, const double& v, const double& w);
Eigen::Matrix<double, 10, 1> monomial_basis_eval(const double& u, const double& v, const double& w);

Eigen::Vector3d CT_eval(const double& u, const double& v, const Eigen::Matrix<double, 12, 3>& dofs);


} // namespace wmtk::components::c1_simplification