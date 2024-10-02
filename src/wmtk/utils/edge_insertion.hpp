#pragma once

#include <wmtk/Types.hpp>
#include "Rational.hpp"

namespace wmtk::utils {

bool segment_intersection_rational(
    const Vector2r& p1,
    const Vector2r& p2,
    const Vector2r& q1,
    const Vector2r& q2,
    Vector2r& intersection);

} // namespace wmtk::utils