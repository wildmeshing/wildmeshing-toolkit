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
 * Generate 100 random points in 2D square domain
 *
 * @returns A vector of Point2D type.
 *
 */
auto pntgen() -> const std::vector<Point2D>&;
auto tagassign(std::vector<Triangle> triangles)
    -> std::map<size_t, size_t>&;
} // namespace wmtk