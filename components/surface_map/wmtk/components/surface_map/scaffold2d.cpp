#include "scaffold2d.hpp"

// Its own translation unit: the arrangement's header brings IPs into the global namespace.
#include <VolumeRemesher/2d/arrangement2d.h>

namespace wmtk::components::surface_map {

bool box_scaffold(
    const std::vector<Eigen::Vector2d>& polygon,
    const Eigen::Vector2d& box_min,
    const Eigen::Vector2d& box_max,
    std::vector<Eigen::Vector2d>& corners,
    std::vector<std::array<int, 3>>& triangles)
{
    using namespace vol_rem::vr2d;
    const size_t n = polygon.size();
    corners.clear();
    triangles.clear();
    if (n < 3) return false;

    std::vector<double> coords;
    coords.reserve(2 * (n + 4));
    for (const Eigen::Vector2d& p : polygon) {
        coords.push_back(p.x());
        coords.push_back(p.y());
    }
    for (const Eigen::Vector2d& c :
         {box_min,
          Eigen::Vector2d(box_max.x(), box_min.y()),
          box_max,
          Eigen::Vector2d(box_min.x(), box_max.y())}) {
        coords.push_back(c.x());
        coords.push_back(c.y());
    }
    std::vector<uint32_t> segments;
    segments.reserve(2 * n);
    for (size_t i = 0; i < n; ++i) {
        segments.push_back(uint32_t(i));
        segments.push_back(uint32_t((i + 1) % n));
    }

    const uint64_t stalls = flip_stall_count();
    Arrangement2D A;
    if (!build_arrangement(coords, segments, A, false)) return false;
    // A stalled insertion leaves a polygon edge out; a crossing adds a vertex.
    if (flip_stall_count() != stalls || A.V.size() != A.num_explicit_pts) return false;

    // Arrangement vertex -> scaffold vertex: the polygon's first, distinct, then the corners.
    std::vector<int> id(A.V.size(), -1);
    for (size_t i = 0; i < n; ++i) {
        const uint32_t v = A.input_point_vertex[i];
        if (v == INVALID || id[v] >= 0) return false;
        id[v] = int(i);
    }
    for (uint32_t v = 0; v < A.num_explicit_pts; ++v) {
        if (id[v] >= 0) continue;
        id[v] = int(n + corners.size());
        corners.emplace_back(A.coords[2 * v], A.coords[2 * v + 1]);
    }

    // The outside: from the arrangement's own corners, never across a polygon edge.
    const uint32_t n_tris = A.num_triangles();
    std::vector<char> outside(n_tris, 0);
    std::vector<uint32_t> stack;
    for (uint32_t t = 0; t < n_tris; ++t) {
        if (!A.tri_is_finite(t)) continue;
        for (int k = 0; k < 3; ++k) {
            if (A.tri_node[3 * t + k] >= A.num_input_pts) {
                outside[t] = 1;
                stack.push_back(t);
                break;
            }
        }
    }
    while (!stack.empty()) {
        const uint32_t t = stack.back();
        stack.pop_back();
        for (int le = 0; le < 3; ++le) {
            if (A.tri_cons[3 * t + le]) continue;
            const uint32_t u = A.tri_neigh[3 * t + le];
            if (u == INVALID || u >= n_tris || outside[u] || !A.tri_is_finite(u)) continue;
            outside[u] = 1;
            stack.push_back(u);
        }
    }
    for (uint32_t t = 0; t < n_tris; ++t) {
        if (!outside[t]) continue;
        triangles.push_back(
            {{id[A.tri_node[3 * t]], id[A.tri_node[3 * t + 1]], id[A.tri_node[3 * t + 2]]}});
    }
    return true;
}

} // namespace wmtk::components::surface_map
