#pragma once

#include <vector>
#include <wmtk/Tuple.hpp>


namespace wmtk::simplex {

/**
 * @brief Compute all tuples that contain simplex(simplex_ptype, t) and simplex(face_ptype, t).
 *
 * face_ptype must be a face of simplex_ptype.
 *
 * The return tuples are guaranteed to contain both input simplices.
 */
std::vector<Tuple> face_coface_intersection(
    const Mesh& mesh,
    const Tuple& t,
    const PrimitiveType simplex_ptype,
    const PrimitiveType face_ptype);
} // namespace wmtk::simplex