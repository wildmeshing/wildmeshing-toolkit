#pragma once

#include <SimpleBVH/BVH.hpp>
#include <wmtk/Types.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

/**
 * Helpers of the geometric tight seal, SimWildMeshTri::tight_seal_geom (AnnotationsTri.cpp) and
 * SimWildMesh::tight_seal_geom (AnnotationsTet.cpp), which keep the operation itself. The helpers
 * below need no mesh; those that do are private members of the two mesh classes, defined in
 * TightSealGeom.cpp.
 */
namespace wmtk::components::simwild::tight_seal {

/// Whether `tag` is the tag named "ambient".
bool is_ambient(const std::map<int64_t, std::string>& tag_id_to_name, int64_t tag);

/// BVH of segments, given as consecutive pairs of points. Returns false if there are none.
bool segments_bvh(const std::vector<Vector2d>& vertices, SimpleBVH::BVH& bvh);
/// BVH of triangles, given as consecutive triples of points. Returns false if there are none.
bool triangles_bvh(const std::vector<Vector3d>& vertices, SimpleBVH::BVH& bvh);

/// The distance from p to the BVH's primitives.
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector2d& p);
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector3d& p);

/**
 * @brief Sphere tracing of a 1-Lipschitz function f along a + t dir, t in [0, L), towards the
 * level set f = level.
 *
 * From a point at f, the level set is at least |f - level| away in every direction, so a step
 * that long along the segment cannot cross it. `from_below`: f(a) < level and each step is
 * level - f; otherwise f(a) > level and each step is f - level. Returns the first t (of a + t dir)
 * where |f - level| <= tol, or nothing if the trace leaves [0, L) first: past the far end, the
 * level set is not on the segment; behind a, f(a) is already beyond the level.
 */
std::optional<double> sphere_trace(
    const std::function<double(const Vector2d&)>& f,
    const Vector2d& a,
    const Vector2d& dir,
    double L,
    double level,
    double tol,
    bool from_below);
std::optional<double> sphere_trace(
    const std::function<double(const Vector3d&)>& f,
    const Vector3d& a,
    const Vector3d& dir,
    double L,
    double level,
    double tol,
    bool from_below);

/// The region i among `regions` whose BVH (bvhs[i]) is nearest p; the first in `regions` on a
/// tie.
size_t nearest_region(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector2d& p,
    const std::vector<size_t>& regions);
size_t nearest_region(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector3d& p,
    const std::vector<size_t>& regions);

/// The six edges of a tet, as pairs of its local vertex indices.
inline constexpr std::array<std::array<int, 2>, 6> tet_edges = {
    {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};

/// Appends the convex polygon `poly`, fanned from its first corner, to `triangles` (consecutive
/// triples of points).
void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles);

/// The distance from p to a closed cell: a triangle in 2D, a tet in 3D; 0 inside it.
double point_cell_distance(const Vector2d& p, const std::array<Vector2d, 3>& cell);
double point_cell_distance(const Vector3d& p, const std::array<Vector3d, 4>& cell);
/// The distance from p to the closed triangle (a, b, c) in 3D.
double
point_triangle_distance(const Vector3d& p, const Vector3d& a, const Vector3d& b, const Vector3d& c);

/// Relative margin of certify_on_cell's tests, so that rounding can only refute a cell.
inline constexpr double certify_margin = 1e-12;
/// certify_on_cell's subdivision levels in 2D and 3D, past which an undecided cell is refuted.
inline constexpr int certify_depth_2d = 8;
inline constexpr int certify_depth_3d = 5;

/**
 * @brief Certifies that a 1-Lipschitz function f stays on one side of `level` over a whole cell
 * (a triangle in 2D, a tet in 3D), conservatively: true only if proven.
 *
 * at_most: max over the cell of f <= level; otherwise: min over the cell of f > level. The
 * conservative circle (sphere) subdivision of topological_offset's Circle.hpp and Sphere.hpp: a
 * circle of radius rho around c holds only values within rho of f(c). Starting from the circle
 * around the cell's bounding square (cube), a circle whose values are all on the right side
 * (by the relative margin certify_margin) is certified; the cell is refuted if a corner is not
 * (corner_values, with the margin), or if a circle's centre lies in the cell with f on the wrong
 * side; any other circle is refined into the circles around its square's 2^dim sub-squares that
 * overlap the cell, and refutes the cell once `max_depth` levels down.
 */
bool certify_on_cell(
    const std::array<Vector2d, 3>& cell,
    const std::array<double, 3>& corner_values,
    const std::function<double(const Vector2d&)>& f,
    double level,
    bool at_most,
    int max_depth);
bool certify_on_cell(
    const std::array<Vector3d, 4>& cell,
    const std::array<double, 4>& corner_values,
    const std::function<double(const Vector3d&)>& f,
    double level,
    bool at_most,
    int max_depth);

/// Sets `fn` to nullptr when it goes out of scope: m_voronoi_split_fn must not outlive the
/// operation that set it.
template <typename Fn>
struct ClearOnExit
{
    Fn& fn;
    ~ClearOnExit() { fn = nullptr; }
};

} // namespace wmtk::components::simwild::tight_seal
