#pragma once

#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>


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

/**
 * @brief Compute all simplices that contain simplex(ptype1, t) and that are contained by
 * simplex(ptype2, t). Simplices can appear multiple times.
 *
 * If ptype1 and ptype2 are the same, only the input tuple is returned.
 *
 * The return tuples are guaranteed to contain both input simplices.
 */
void simplices_preserving_primitive_types(
    SimplexCollection& collection,
    const Tuple& t,
    const PrimitiveType ptype1,
    const PrimitiveType ptype2,
    const PrimitiveType pt_return);
} // namespace wmtk::simplex