#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::components::internal {

Eigen::VectorXd winding_number(const Mesh& m, const TriMesh& surface);

Eigen::VectorXd winding_number_internal(
    const Eigen::MatrixXd& query_points,
    const Eigen::MatrixXd& surface_V,
    const MatrixX<int64_t>& surface_F);

} // namespace wmtk::components::internal