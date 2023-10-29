#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Returns a vector with all vertices in the boundary of a simplex.
 *
 * The simplices are sorted according to the input tuple.
 *
 * Vertex: 0 -->
 *
 * Edge: 0 --> 1
 *
 * Triangle:
 *      2
 *     / \ .
 *    / x \ .
 *   0 --> 1
 *
 * Tetrahedra:
 *        2
 *       / \\ .
 *      /   \ \ .
 *     /  x  \  \ .
 *    /       \   \ 3
 *   0 -->---- 1
 *
 * @param mesh The mesh containing the simplex
 * @param simplex The simplex
 *
 * @return A vector of vertices sorted according to the tuple orientation of the simplex
 */
std::vector<Tuple> vertices(const Mesh& mesh, const Simplex& simplex);
} // namespace wmtk::simplex