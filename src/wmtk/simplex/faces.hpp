#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Returns all faces of a simplex.
 *
 * The faces are all lower dimensional simplices that bound the given k-simplex.
 * This does not include the given simplex itself!
 *
 * - tetrahedron: 4 faces, 6 edges, 4 vertices
 * - face: 3 edges, 3 vertices
 * - edge: 2 vertices
 * - vertex: nothing
 *
 * @param mesh The mesh containing the simplex
 * @param simplex The simplex
 * @param sort_and_clean Call `SimplexCollection::sort_and_clean` before returning
 *
 * @return `SimplexCollection` holding the faces
 */
SimplexCollection faces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

void faces(
    SimplexCollection& simplex_collection,
    const Simplex& simplex,
    const bool sort_and_clean = true);
} // namespace wmtk::simplex
