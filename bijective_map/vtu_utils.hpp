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

/**
 * @brief Write a tetrahedral mesh to VTU format
 * @param V Vertex coordinates matrix (n x 3)
 * @param T Tetrahedron indices matrix (m x 4)
 * @param filename Output filename
 */
void write_tet_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const std::string& filename);

/**
 * @brief Write an edge mesh to VTU format
 * @param V Vertex coordinates matrix (n x 2 or n x 3)
 * @param E Edge indices matrix (m x 2)
 * @param filename Output filename
 */
void write_edge_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& E,
    const std::string& filename);

/**
 * @brief Read triangle mesh from VTU file
 * @param filename VTU filename
 * @param V Output vertex coordinates matrix (n x 3)
 * @param F Output face indices matrix (m x 3)
 * @return true if read successful, false otherwise
 */
bool read_triangle_mesh_from_vtu(
    const std::string& filename,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F);


} // namespace vtu_utils