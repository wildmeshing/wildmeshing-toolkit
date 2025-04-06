#pragma once

#include <vector>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}

namespace wmtk::simplex {

/**
 * @brief Compute all tuples that contain simplex(ptype1, t) and that are contained by
 * simplex(ptype2, t).
 *
 * If ptype1 and ptype2 are the same, only the input tuple is returned.
 *
 * The return tuples are guaranteed to contain both input simplices.
 */
std::vector<Tuple> tuples_preserving_primitive_types(
    const Mesh& mesh,
    const Tuple& t,
    const PrimitiveType ptype1,
    const PrimitiveType ptype2);
} // namespace wmtk::simplex