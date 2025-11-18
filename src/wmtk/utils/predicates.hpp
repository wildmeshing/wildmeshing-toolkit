#pragma once

#include <igl/predicates/predicates.h>
#include <wmtk/Types.hpp>

namespace wmtk::utils::predicates {

/**
 * @brief Check if three vertices in 2D are collinear using exact predicates.
 */
inline bool is_degenerate(const Vector2d& v0, const Vector2d& v1, const Vector2d& v2)
{
    return igl::predicates::orient2d(v0, v1, v2) == igl::predicates::Orientation::COLLINEAR;
}

/**
 * @brief Check if three vertices in 3D are collinear using exact predicates.
 */
inline bool is_degenerate(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2)
{
    for (int dim = 0; dim < 3; ++dim) {
        const Vector2d p0(v0[dim], v0[(dim + 1) % 3]);
        const Vector2d p1(v1[dim], v1[(dim + 1) % 3]);
        const Vector2d p2(v2[dim], v2[(dim + 1) % 3]);

        if (!is_degenerate(p0, p1, p2)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if four vertices in 3D are coplanar using exact predicates.
 */
inline bool
is_degenerate(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)
{
    return igl::predicates::orient3d(v0, v1, v2, v3) == igl::predicates::Orientation::COPLANAR;
}

} // namespace wmtk::utils::predicates