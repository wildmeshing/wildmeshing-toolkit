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

/// Sets `fn` to nullptr when it goes out of scope: m_voronoi_split_fn must not outlive the
/// operation that set it.
template <typename Fn>
struct ClearOnExit
{
    Fn& fn;
    ~ClearOnExit() { fn = nullptr; }
};

} // namespace wmtk::components::simwild::tight_seal
