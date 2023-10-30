#pragma once

#include <wmtk/Primitive.hpp>

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief Returns a vector with all vertices in the boundary of a simplex.
 *
 * The vertices are sorted according to the input tuple:
 * v0 = simplex.tuple()
 * v1 = mesh.switch_vertex(v0);
 * v2 = mesh.switch_vertex(mesh.switch_edge(v0));
 * v3 = mesh.switch_vertex(mesh.switch_edge(mesh.switch_face(v0)));
 *
 * Vertex:     {v0}
 * Edge:       {v0, v1}
 * Triangle:   {v0, v1, v2}
 * Tetrahedra: {v0, v1, v2, v3}
 *
 * @param mesh The mesh containing the simplex
 * @param simplex The simplex
 *
 * @return A vector of vertices sorted according to the tuple orientation of the simplex
 */
std::vector<Tuple>
faces_single_dimension(const Mesh& mesh, const Simplex& simplex, const PrimitiveType face_type);
} // namespace wmtk::simplex