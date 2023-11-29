#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Returns all boundary simplices of a simplex.
 *
 * The boundary simplices are the k-1 simplices that bound the given k-simplex.
 * This does not include the given simplex itself!
 *
 * - tetrahedron: 4 faces
 * - face: 3 edges
 * - edge: 2 vertices
 * - vertex: nothing
 *
 * @param mesh The mesh containing the simplex
 * @param simplex The simplex
 * @param sort_and_clean Call `SimplexCollection::sort_and_clean` before returning
 *
 * @return `SimplexCollection` holding the boundary
 */
SimplexCollection
boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

std::vector<Tuple> boundary_tuples(const Mesh& mesh, const Simplex& simplex);
std::vector<Tuple> boundary_tuples(const Mesh& mesh, const Tuple& t, PrimitiveType pt);
} // namespace wmtk::simplex
