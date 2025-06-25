#pragma once

#include <Eigen/Dense>
#include <string>

namespace vtu_utils {

/**
 * @brief Write a triangle mesh to VTU format
 * @param V Vertex coordinates matrix (n x 3)
 * @param F Face indices matrix (m x 3)
 * @param filename Output filename
 */
void write_triangle_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const std::string& filename);

/**
 * @brief Write a point mesh to VTU format
 * @param V Vertex coordinates matrix (n x 3)
 * @param filename Output filename
 */
void write_point_mesh_to_vtu(const Eigen::MatrixXd& V, const std::string& filename);

} // namespace vtu_utils