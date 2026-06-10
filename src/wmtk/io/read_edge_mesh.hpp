#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::io {

/**
 * @brief Reads an edge mesh from a file.
 * The mesh is cleaned by removing duplicated vertices, degenerate faces, and unreferenced vertices.
 *
 * Duplicated vertices are removed based on the provided tolerances. The tolerance can be specified
 * in two ways: absolute and relative. Only one of them can be non-negative. If both are negative,
 * duplicated vertices will not be removed.
 *
 * @param path The file path to read the mesh from.
 * @param V Output vertex positions. Size is #V by 3.
 * @param E Output edge indices. Size is #E by 2.
 */
void read_edge_mesh(const std::string& path, MatrixXd& V, MatrixXi& E);

} // namespace wmtk::io