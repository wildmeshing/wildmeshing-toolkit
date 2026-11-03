#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace wmtk::components::tracking {

/// A positively oriented triangle in the plane. Triangles of one triangulation name a shared
/// corner by the same point, bit for bit: that is how a walk finds its way between them.
using Triangle2 = std::array<Eigen::Vector2d, 3>;

/// A point on a triangle, by barycentric coordinates. A point on an edge or a corner of it has
/// exact zeros, and a walk leaves those where they are.
struct WalkPoint
{
    size_t tri;
    Eigen::Vector3d bary;
};

/// The part of a segment on one triangle, from one end to the other.
struct WalkPiece
{
    size_t tri;
    std::array<Eigen::Vector3d, 2> bary;
};

/// The point `l` of triangle `t` in the coordinates of triangle `n`, if `n` has the corners it is
/// on: a point on a shared edge or corner, which keeps its coordinates and its exact zeros.
std::optional<Eigen::Vector3d>
carry(const Triangle2& t, const Eigen::Vector3d& l, const Triangle2& n);

/**
 * @brief The straight segment between two points of a triangulation, cut at its edges.
 *
 * Which edges and corners the segment goes through is decided by exact orient2d alone, on the
 * two ends and the triangles' corners, so it needs no tolerance and rounding cannot lead it
 * astray; the doubles only say where along an edge a crossing is. A piece that crosses a corner
 * or runs along an edge is handled as such: its ends have exact zeros.
 *
 * Empty if the walk finds no way: a triangle that is not positively oriented, or a segment that
 * leaves the triangulation.
 */
std::optional<std::vector<WalkPiece>>
walk_segment(const std::vector<Triangle2>& tris, const WalkPoint& from, const WalkPoint& to);

} // namespace wmtk::components::tracking
