#pragma once

#include "ConnectedComponent.hpp"

#include <SimpleBVH/BVH.hpp>
#include <wmtk/Types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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

/// BVH of segments, given as consecutive pairs of points. Returns false if there are none.
bool segments_bvh(const std::vector<Vector2d>& vertices, SimpleBVH::BVH& bvh);
/// BVH of triangles, given as consecutive triples of points. Returns false if there are none.
bool triangles_bvh(const std::vector<Vector3d>& vertices, SimpleBVH::BVH& bvh);

/// The distance from p to the BVH's primitives.
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector2d& p);
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector3d& p);

/**
 * @brief The point of a simplex where k distance functions, each interpolated linearly from its
 * values at the simplex's k corners, are all equal: for a triangle (k = 3) the triple point of
 * three regions, for a tet (k = 4) the quadruple point of four (tight seal v2 note, section 4a-i).
 *
 * D(r, c) is function r at corner c. Returns the point's barycentric coordinates (they sum to 1),
 * or nothing if the system is singular. The point lies inside the simplex if all are > 0.
 */
std::optional<Eigen::VectorXd> equal_distances_point(const Eigen::MatrixXd& D);

/**
 * @brief Where a 1-Lipschitz f reaches `level` along the segment [a, b], coming from a, which is
 * below the level if `from_below` (f(a) < level), else above it (f(a) > level).
 *
 * Sphere traced from a, each step |f - level| + step_floor (at least step_floor, so the trace ends;
 * no step goes more than step_floor past the level), until f is at the level or past it; the last
 * step is then bisected to the level. Nothing if f(a) is not on the side `from_below` gives, or if
 * f stays on that side up to b.
 */
template <typename V>
std::optional<V> level_crossing(
    const std::function<double(const V&)>& f,
    const V& a,
    const V& b,
    const double level,
    const double step_floor,
    const bool from_below)
{
    const auto before = [&](const double v) { return from_below ? v < level : v > level; };
    double v = f(a);
    if (!before(v)) {
        return std::nullopt;
    }
    const double L = (b - a).norm();
    const V dir = (b - a) / L;
    double t_prev = 0.;
    double t = 0.;
    while (before(v)) {
        t_prev = t;
        t += std::abs(v - level) + step_floor;
        if (t >= L) {
            t = L;
            v = f(b);
            if (before(v)) {
                return std::nullopt;
            }
            break;
        }
        v = f(a + t * dir);
    }
    // f(t_prev) before the level, f(t) at it or past it
    double lo = t_prev;
    double hi = t;
    for (int i = 0; i < 50; ++i) {
        const double m = (lo + hi) / 2;
        if (before(f(a + m * dir))) {
            lo = m;
        } else {
            hi = m;
        }
    }
    return V(a + ((lo + hi) / 2) * dir);
}

/// The point of the segment [a, b] where f changes sign, by bisection, given f(a) < 0 <= f(b).
template <typename V>
V bisect_sign_change(const std::function<double(const V&)>& f, V a, V b)
{
    for (int i = 0; i < 50; ++i) {
        const V m = (a + b) / 2;
        if (f(m) < 0) {
            a = m;
        } else {
            b = m;
        }
    }
    return (a + b) / 2;
}

/// The six edges of a tet, as pairs of its local vertex indices.
inline constexpr std::array<std::array<int, 2>, 6> tet_edges = {
    {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};

/// Appends the convex polygon `poly`, fanned from its first corner, to `triangles` (consecutive
/// triples of points).
void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles);

/**
 * @brief The region "split_fill" gives a cell, from its corners' own labels.
 *
 * The corners' region if they agree. Otherwise an edge between two of them was not split, because
 * the distances to their regions are equal at one of its ends; the cell then takes the one region
 * that every corner's tie set holds. If they share several (an exact tie between regions) or none,
 * the choice is arbitrary: the first region they share, or else the first corner's.
 *
 * ties(v): the regions within the tie tolerance of the nearest one at v, ascending. Returns the
 * region and how it was chosen: 0 the corners agree, 1 their tie sets, 2 arbitrarily.
 */
std::pair<size_t, int> split_fill_label(
    const CornerLabels& corners,
    const std::function<const std::vector<size_t>&(size_t)>& ties);

/// The pairs of `parent` whose vertex is one of `corners`: a child's labels after a split.
template <typename Corners>
CornerLabels kept_corner_labels(const CornerLabels& parent, const Corners& corners)
{
    CornerLabels kept;
    for (const auto& [v, region] : parent) {
        if (std::find(corners.begin(), corners.end(), v) != corners.end()) {
            kept.emplace_back(v, region);
        }
    }
    return kept;
}

/// Sets `fn` to nullptr when it goes out of scope: m_voronoi_split_fn must not outlive the
/// operation that set it.
template <typename Fn>
struct ClearOnExit
{
    Fn& fn;
    ~ClearOnExit() { fn = nullptr; }
};

} // namespace wmtk::components::simwild::tight_seal
