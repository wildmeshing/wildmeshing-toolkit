#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::utils {

/**
 * @brief Compute the bounding box of a mesh.
 *
 * @param position_handle The position handle, which also contains a pointer to the mesh.
 * @return A matrix where the first row contains all minimal coordinates and the second row contains
 * all maximal coordinates.
 */
Eigen::MatrixXd bbox_from_mesh(const attribute::MeshAttributeHandle& position_handle);

/**
 * @brief Compute the diagonal of the bounding box of a mesh.
 * @param position_handle The position handle, which also contains a pointer to the mesh.
 * @return The diagonal of the bounding box.
 */
double bbox_diagonal_from_mesh(const attribute::MeshAttributeHandle& position_handle);

} // namespace wmtk::utils