
#pragma once

#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
}


namespace wmtk::simplex::internal {
/**
 * @brief Returns the faces (single-dimension) of a simplex that are also cofaces of a given face.
 *
 *
 * Say the l-simplex B is a face of a k-simplex S, then this function returns the faces of S that
 * are also cofaces of B. B and S are encoded within a single tuple t and two primitive types pt and
 * face_pt. The returned tuples always contain B.
 *
 * This function supports cofaces_single_dimension.
 *
 * Example 1: given a triangle and a vertex that is a face of the triangle, this function computes
 * the edges that are faces of the triangle and also cofaces of the vertex.
 *
 * Example 2: given a tetrahedron and a vertex that is a face of the tetrahedron, this function
 * computes the triangles that are faces of the tetrahedron and also cofaces of the vertex.
 *
 * Example 3: given a tetrahedron and an edge that is a face of the tetrahedron, this function
 * computes the triangles that are faces of the tetrahedron and also cofaces of the edge.
 *
 *
 * @param mesh The mesh containing the simplex.
 * @param tuple The tuple of the simplex whose faces we are computing.
 * @param pt Primitive type of the simplex whose faces we are computing.
 * @param face_pt Primitive type of the face being preserved.
 *
 * @return Faces of the simplex(t,pt) that also are cofaces of the simplex(t, face_pt).
 */
std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt);


/**
 * @brief Returns the faces (single-dimension) of a simplex that are also cofaces of a given face.
 *
 * @param mesh The mesh containing the simplex.
 * @param simplex The simplex whose faces we are computing.
 * @param face_pt Primitive type of the face being preserved.
 *
 * @return Faces of simplex that also are cofaces of the simplex(simplex.tuple(), face_pt).
 */
std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType face_pt);

/**
 * @brief Returns the faces (single-dimension) of a simplex that are also cofaces of a given face.
 *
 *
 * Say the l-simplex B is a face of a k-simplex S, then this function returns the faces of S that
 * are also cofaces of B. B and S are encoded within a single tuple t and two primitive types pt and
 * face_pt. The returned tuples always contain B.
 *
 * This function supports cofaces_single_dimension.
 *
 *
 * @param mesh The mesh containing the simplex.
 * @param tuple The tuple of the simplex whose faces we are computing.
 * @param pt Primitive type of the simplex whose faces we are computing.
 * @param face_pt Primitive type of the face being preserved.
 *
 * @return Faces of the simplex(t,pt) that also are cofaces of the simplex(t, face_pt).
 */
std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt);


/**
 * @brief Returns the faces (single-dimension) of a simplex that are also cofaces of a given face.
 *
 * @param mesh The mesh containing the simplex.
 * @param simplex The simplex whose faces we are computing.
 * @param face_pt Primitive type of the face being preserved.
 *
 * @return Faces of simplex that also are cofaces of the simplex(simplex.tuple(), face_pt).
 */
std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType face_pt);
} // namespace wmtk::simplex::internal
