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
Eigen::Matrix<double, 10, 2> monomial_basis_grad(const double& u, const double& v);

Eigen::Vector3d CT_eval(const double& u, const double& v, const Eigen::Matrix<double, 12, 3>& dofs);
Eigen::Matrix<double, 3, 2>
CT_grad(const double& u, const double& v, const Eigen::Matrix<double, 12, 3>& dofs);

void read_tetmesh(const std::string& filename, Eigen::MatrixXd& V, Eigen::MatrixXi& T);

} // namespace wmtk::components::c1_simplification