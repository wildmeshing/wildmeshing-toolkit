#pragma once

#include <wmtk/Primitive.hpp>

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Returns a vector with all faces in the boundary of a simplex of the given dimension.
 *
 * Assuming the following tetrahedron:
 *
 *           0
 *          / \\ .
 *         /   \  \ .
 *        /     \   \ .
 *       /       \     \ .
 *      /         \      3
 *     /           \    /
 *    /             \  /
 *   1 ------------- 2
 *
 *
 * Given the tuple representing v(0), e(0,1), f(0,1,2), t(0,1,2,3) we have the following faces:
 *
 * Tetrahedron:
 *
 *   PrimitiveType::Face
 *   {
 *     f(0,1,2),
 *     f(0,3,1),
 *     f(1,3,2),
 *     f(2,3,0)
 *   }
 *
 *   PrimitiveType::Edge
 *   {
 *     e(0,1),
 *     e(1,2),
 *     e(2,0),
 *     e(0,3),
 *     e(1,3),
 *     e(2,3)
 *   }
 *
 *   PrimitiveType::Vertex
 *   {v(0),v(1),v(2),v(3)}
 *
 * Triangle:
 *
 *   PrimitiveType::Edge
 *   {
 *     e(0,1),
 *     e(1,2),
 *     e(2,0)
 *   }
 *
 *   PrimitiveType::Vertex
 *   {v(0),v(1),v(2)}
 *
 * Edge:
 *
 *   PrimitiveType::Vertex
 *   {v(0),v(1)}
 *
 * The order is relative to the orientation of the input tuple and guaranteed to be always the same
 * (i.e. it is independent of the internal indices).
 *
 *
 * @param mesh The mesh containing the simplex
 * @param simplex The simplex
 * @param face_type The requested face type
 *
 * @return A vector of vertices sorted according to the tuple orientation of the simplex
 */
SimplexCollection
faces_single_dimension(const Mesh& mesh, const Simplex& simplex, const PrimitiveType face_type);

void faces_single_dimension(
    SimplexCollection& simplex_collection,
    const Simplex& simplex,
    const PrimitiveType face_type);

std::vector<Tuple> faces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType face_type);
} // namespace wmtk::simplex