#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/Rational.hpp>


namespace wmtk::components::internal {

/**
 * @brief
 *
 * @param point
 * @param t1
 * @param t2
 * @param t3
 * @return int -1: outside 0: inside 1: on AB 2: on BC 3: on AC 4:on endpoint
 */
int is_point_inside_triangle(
    const wmtk::Vector2r& P,
    const wmtk::Vector2r& A,
    const wmtk::Vector2r& B,
    const wmtk::Vector2r& C);


/**
 * @brief include the intersection on edgepoints
 *
 */
bool segment_segment_inter(
    const Vector2r& s0,
    const Vector2r& e0,
    const Vector2r& s1,
    const Vector2r& e1,
    Vector2r& res);

} // namespace wmtk::components::internal