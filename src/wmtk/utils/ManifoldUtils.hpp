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

namespace manifold_internal {

using Vertices = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Facets = Eigen::Matrix<uint64_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

void resolve_nonmanifoldness(
    Vertices& V,
    Facets& F,
    std::vector<size_t>& modified_vertices);

} // namespace manifold_internal

} // namespace wmtk
