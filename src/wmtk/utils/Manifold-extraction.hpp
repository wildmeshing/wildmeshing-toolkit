#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <cstdlib>
#include <random>
#include <map>
#include <memory>


namespace wmtk {
using Point3D = std::array<double, 3>;
using Tetrahedron = std::array<size_t, 4>;
using Point2D = std::array<double, 2>;
using Triangle = std::array<size_t, 3>;

/**
 * Generate 100 random points in 2D square domain
 *
 * @returns A vector of Point2D type.
 *
 */
auto pntgen2d(const size_t nb_points, const double range) -> std::vector<Point2D>;
auto pntgen3d(size_t nb_points, double range) -> std::vector<Point3D>;
auto tagassign(size_t nb_triangles) -> std::vector<size_t>;
} // namespace wmtk