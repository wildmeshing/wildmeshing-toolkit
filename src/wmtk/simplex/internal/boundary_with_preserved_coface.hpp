
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
 * @param tuple The tuple of the simplex
 * @param pt primitive type of the simplex
 * @param coface_pt primitive type of the coface
 *
 * @return boundary of the tuple
 */
std::vector<Tuple> coface_preserving_boundary_tuples(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType coface_pt);
} // namespace wmtk::simplex::internal
