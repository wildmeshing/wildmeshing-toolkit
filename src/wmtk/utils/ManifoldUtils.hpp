#pragma once


#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <array>

namespace wmtk {
// convert triangle soup to manifold components for TriMesh.
bool separate_to_manifold(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Eigen::Vector3d>& out_v,
    std::vector<std::array<size_t, 3>>& out_f,
    std::vector<size_t>& modified_vertices);

void resolve_nonmanifoldness(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    std::vector<size_t>& modified_vertices);
} // namespace wmtk