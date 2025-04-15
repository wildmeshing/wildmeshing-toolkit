#pragma once

#include <Eigen/Core>
#include <unordered_set>
#include <vector>

namespace wmtk {
namespace operations {
namespace utils {

/**
 * @brief Analyzes tetrahedral mesh and boundary faces to identify interior vertices
 * @param T Connectivity information of the tetrahedral mesh
 * @param F_bd Connectivity information of the boundary faces
 * @return List of indices of interior vertices
 */
std::vector<int> embed_mesh(const Eigen::MatrixXi& T, const Eigen::MatrixXi& F_bd);

/**
 * @brief Finds all triangular faces formed by interior vertices
 * @param non_bd_vertices Set of interior vertices
 * @param T Connectivity information of the tetrahedral mesh
 * @return Matrix of triangular faces formed by interior vertices
 */
Eigen::MatrixXi find_F_top(
    const std::unordered_set<int>& non_bd_vertices,
    const Eigen::MatrixXi& T);

Eigen::MatrixXd parametrize_top(
    const Eigen::MatrixXi& F_top,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F_bd,
    const Eigen::MatrixXd& uv_bd,
    const Eigen::VectorXi& IM_uv_bd,
    const Eigen::MatrixXi& T,
    Eigen::VectorXi& IM_top,
    bool debug_mode = false);

void visualize_tet_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& T);

void launch_debug_viewer(
    const Eigen::MatrixXd& V_bottom,
    const Eigen::MatrixXi& F_bottom,
    const Eigen::MatrixXd& uv_bottom);

} // namespace utils
} // namespace operations
} // namespace wmtk