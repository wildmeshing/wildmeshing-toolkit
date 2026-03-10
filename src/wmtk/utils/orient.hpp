#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::utils {

bool orient3d(const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3);

bool orient3d(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, const Vector3d& p3);

bool orient2d(const Vector2r& p0, const Vector2r& p1, const Vector2r& p2);

bool orient2d(const Vector2d& p0, const Vector2d& p1, const Vector2d& p2);

} // namespace wmtk::utils