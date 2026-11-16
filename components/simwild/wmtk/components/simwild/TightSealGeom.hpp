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

/// The six edges of a tet, as pairs of its local vertex indices.
inline constexpr std::array<std::array<int, 2>, 6> tet_edges = {
    {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};

/// Appends the convex polygon `poly`, fanned from its first corner, to `triangles` (consecutive
/// triples of points).
void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles);

/// Empties `fn` (a std::function or a std::optional) when it goes out of scope:
/// m_voronoi_split_fn and m_edge_split_at must not outlive the operation that set them.
template <typename Fn>
struct ClearOnExit
{
    Fn& fn;
    ~ClearOnExit() { fn = Fn{}; }
};

} // namespace wmtk::components::simwild::tight_seal
