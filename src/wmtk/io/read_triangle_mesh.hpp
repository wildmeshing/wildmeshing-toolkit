#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::io {

/**
 * @brief Reads a triangle mesh from a file.
 * The mesh is cleaned by removing duplicated vertices, degenerate faces, and unreferenced vertices.
 *
 * Duplicated vertices are removed based on the provided tolerances. The tolerance can be specified
 * in two ways: absolute and relative. Only one of them can be non-negative. If both are negative,
 * duplicated vertices will not be removed.
 *
 * @param path The file path to read the mesh from.
 * @param V Output vertex positions. Size is #V by 3.
 * @param F Output face indices. Size is #F by 3.
 * @param tol_abs Absolute tolerance for removing duplicated vertices. If negative,
 * duplicated vertices will not be removed.
 * @param tol_rel Relative tolerance for removing duplicated vertices. If negative,
 * duplicated vertices will not be removed.
 */
void read_triangle_mesh(
    const std::string& path,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    double tol_rel = 2e-4,
    double tol_abs = -1);

void read_triangle_mesh(
    const std::vector<std::string>& paths,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    double tol_rel = 2e-4,
    double tol_abs = -1);

} // namespace wmtk::io