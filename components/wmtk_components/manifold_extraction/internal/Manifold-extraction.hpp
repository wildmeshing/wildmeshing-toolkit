#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <cstdlib>
#include <random>
#include <map>

namespace wmtk {
using Point3D = std::array<double, 3>;
using Tetrahedron = std::array<size_t, 4>;
using Point2D = std::array<double, 2>;
using Triangle = std::array<size_t, 3>;

/**
 * Generate nb_points random points in 2D square domain, from 0 to range
 *
 * @returns A vector of Point2D type.
 *
 */
auto pntgen2d(const size_t nb_points, const double range) -> std::vector<Point2D>;

/**
 * Generate nb_points random points in 3D square domain, from 0 to range
 *
 * @returns A vector of Point3D type.
 *
 */
auto pntgen3d(size_t nb_points, double range) -> std::vector<Point3D>;

/**
 * Randomly assign tags to the triangles of size nb_triangles.
 *
 * @returns A vector of size_t type, only keeping the indecies of the tagged ones, prob=50%
 *
 */
auto tagassign(size_t nb_triangles) -> std::vector<size_t>;
} // namespace wmtk