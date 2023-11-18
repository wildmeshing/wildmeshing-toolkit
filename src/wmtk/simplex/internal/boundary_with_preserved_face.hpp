
#pragma once

#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
}


namespace wmtk::simplex::internal {
/**
 * @brief Returns the boundary of a simplex ignoring some sorts of switches
 *
 * Say the input k-simplex S is a coface of a l-simplex B. Then this function returns the boundary
 * simplices of S that are still cofaces. Because preserving being a coface of B does not require
 * tracking B itself, but just its dimension l we pass in a primitive type instead. This function
 * supports cofaces_single_dimension.
 *
 * @param mesh The mesh containing the simplex
 * @param tuple The tuple of the simplex whose face we are computing
 * @param pt primitive type of the simplex whose face we are computing
 * @param face_pt primitive type of the face being preserved
 *
 * @return boundary of the tuple
 */
std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt);


/**
 * @brief Given a simplex and one of its faces, returns
 *
 * boundary_with_preserved_face_tuples
 *
 * @param mesh The mesh containing the simplex
 * @param simplex simplex to compute the boundary of
 * @param face_pt primitive type of the face being preserved
 *
 * @return boundary of the tuple
 */
std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType face_pt);

/**
 * @brief Returns the boundary of a simplex ignoring some sorts of switches
 *
 * Say the input k-simplex S is a coface of a l-simplex B. Then this function returns the boundary
 * simplices of S that are still cofaces. Because preserving being a coface of B does not require
 * tracking B itself, but just its dimension l we pass in a primitive type instead. This function
 * supports cofaces_single_dimension.
 *
 * @param mesh The mesh containing the simplex
 * @param tuple The tuple of the simplex whose face we are computing
 * @param pt primitive type of the simplex whose face we are computing
 * @param face_pt primitive type of the face being preserved
 *
 * @return boundary of the tuple
 */
std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt);


/**
 * @brief Given a simplex and one of its faces, returns
 *
 * boundary_with_preserved_face_tuples
 *
 * @param mesh The mesh containing the simplex
 * @param simplex simplex to compute the boundary of
 * @param face_pt primitive type of the face being preserved
 *
 * @return boundary of the tuple
 */
std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType face_pt);
} // namespace wmtk::simplex::internal
