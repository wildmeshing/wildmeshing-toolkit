#pragma once

#include <array>
#include <cstddef>
#include <vector>
#include <wmtk/Types.hpp>

namespace wmtk::delaunay {
using Point3D = std::array<double, 3>;
using Tetrahedron = std::array<size_t, 4>;
using Point2D = std::array<double, 2>;
using Triangle = std::array<size_t, 3>;

/**
 * Compute the Delaunay tetrahedralization of the input 3D points.
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


/**
 * Compute the Delaunay triangulation of the input 2D points.
 * @param[in] points
 *
 * @returns A tuple of (vertices, tets).
 *
 * Example:
 *     std::vector<Point2D> points;
 *     // Populate points.
 *     auto [vertices, triangles] = delaunay2D(points);
 */
auto delaunay2D(const std::vector<Point2D>& points)
    -> std::pair<std::vector<Point2D>, std::vector<Triangle>>;

/**
 * @brief Compute the constrained Delaunay triangulation of the input 2D points and edges.
 *
 * The mesh must not have any duplicate vertices or edges.
 *
 * @param[in] V The input 2D points.
 * @param[in] E The constrained input edges.
 * @param[out] V_out The output 2D points.
 * @param[out] F_out The output triangles.
 * @param[out] E_out The constrained input edges in the output mesh. They might differ from the
 * input edges if some input edges are split during triangulation.
 */
void constrained_delaunay2D(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out);

} // namespace wmtk::delaunay
