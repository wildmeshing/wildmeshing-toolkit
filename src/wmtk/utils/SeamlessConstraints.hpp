#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <wmtk/TriMesh.hpp>

namespace wmtk::utils {

extern const std::array<Eigen::Matrix<double, 2, 2>, 4> rotation_matrix;

simplex::Simplex get_pair_edge(
    const TriMesh& seamed_mesh,
    const TriMesh& cut_mesh,
    const simplex::Simplex& edge_simplex);

Eigen::Matrix<double, 2, 2> get_rotation_matrix(
    const TriMesh& cut_mesh,
    const TypedAttributeHandle<double>& uv_coordinate,
    const simplex::Simplex& edge_simplex,
    const simplex::Simplex& pair_edge_simplex);

bool check_constraints(
    const TriMesh& seamed_mesh,
    const TriMesh& cut_mesh,
    const TypedAttributeHandle<double>& uv_coordinate,
    double eps = 1e-6);

} // namespace wmtk::utils
