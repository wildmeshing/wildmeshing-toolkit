#pragma once

#include <array>
#include <vector>

namespace wmtk {
using Point3D = std::array<double, 3>;
using Tetrahedron = std::array<size_t, 4>;

/**
 * Compute the Delaunay tetrahedralization of the input points.
 * @param[in] points
 *
 * @returns A tuple of (vertices, tets).
 *
 * Example:
 *     std::vector<Point3D> points;
 *     // Populate points.
 *     auto [vertices, tets] = delaunay3D(points);
 */
auto delaunay3D(const std::vector<Point3D>& points)
    -> std::pair<std::vector<Point3D>, std::vector<Tetrahedron>>;
auto delaunay3D_conn(const std::vector<Point3D>& points)
    -> std::vector<Tetrahedron>;
} // namespace wmtk
