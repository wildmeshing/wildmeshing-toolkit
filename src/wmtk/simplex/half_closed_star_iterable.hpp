#pragma once

#include "iterable/HalfClosedStarIterable.hpp"

namespace wmtk::simplex {

/**
 * The half closed star is used to determine the deleted simplices in an edge collapse.
 *
 * The result is the same as computing the intersection of the open star of the vertex and the
 * closed star of the edge that is represented by the tuple.
 *
 * @param mesh The mesh.
 * @param tuple The tuple representing the edge and vertex that are collapsed and deleted.
 */
HalfClosedStarIterable half_closed_star_iterable(const Mesh& mesh, const Tuple& tuple);

} // namespace wmtk::simplex