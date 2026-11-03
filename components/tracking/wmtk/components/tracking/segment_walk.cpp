#include "segment_walk.hpp"

#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace wmtk::components::tracking {

namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;

/// Which side of the directed line (p, q) `c` is on: 1 left, -1 right, 0 on it. Exact.
int side(const Vector2d& p, const Vector2d& q, const Vector2d& c)
{
    return int(wmtk::utils::predicates::orient2d(p, q, c));
}

Vector2d position(const Triangle2& t, const Vector3d& l)
{
    return l[0] * t[0] + l[1] * t[1] + l[2] * t[2];
}

int zeros(const Vector3d& l)
{
    return int(l[0] == 0) + int(l[1] == 0) + int(l[2] == 0);
}

int corner_of(const Triangle2& t, const Vector2d& c)
{
    for (int k = 0; k < 3; ++k) {
        if (t[k] == c) return k;
    }
    return -1;
}

/// The triangle holding the edge from a to b, in that direction.
std::optional<size_t>
across(const std::vector<Triangle2>& tris, const Vector2d& a, const Vector2d& b)
{
    for (size_t n = 0; n < tris.size(); ++n) {
        const int i = corner_of(tris[n], a);
        if (i >= 0 && tris[n][(i + 1) % 3] == b) return n;
    }
    return std::nullopt;
}

/// Among the triangles around corner `v`, the one the segment goes on through, with its corner.
std::optional<std::pair<size_t, int>> onwards_from_corner(
    const std::vector<Triangle2>& tris,
    const Vector2d& v,
    const Vector2d& p,
    const Vector2d& q)
{
    const Vector2d d = q - p;
    for (size_t n = 0; n < tris.size(); ++n) {
        const int j = corner_of(tris[n], v);
        if (j < 0) continue;
        const Vector2d& a = tris[n][(j + 1) % 3];
        const Vector2d& b = tris[n][(j + 2) % 3];
        const int sa = side(p, q, a), sb = side(p, q, b);
        const bool wedge = sa < 0 && sb > 0;
        const bool along = (sa == 0 && (a - v).dot(d) > 0) || (sb == 0 && (b - v).dot(d) > 0);
        if (wedge || along) return std::make_pair(n, j);
    }
    return std::nullopt;
}

double distance_from_line(const Vector2d& c, const Vector2d& p, const Vector2d& d)
{
    return std::abs(d.x() * (c.y() - p.y()) - d.y() * (c.x() - p.x()));
}

/// The corner next to `v`, ahead of it, nearest the line: where a segment that runs along an edge
/// from `v` goes when rounding puts the line just off it. Its triangle, and `v` and it there.
std::optional<std::tuple<size_t, int, int>> nearest_corner_ahead(
    const std::vector<Triangle2>& tris,
    const Vector2d& v,
    const Vector2d& p,
    const Vector2d& q)
{
    const Vector2d d = q - p;
    std::optional<std::tuple<size_t, int, int>> best;
    double nearest = std::numeric_limits<double>::infinity();
    for (size_t n = 0; n < tris.size(); ++n) {
        const int j = corner_of(tris[n], v);
        if (j < 0) continue;
        for (const int c : {(j + 1) % 3, (j + 2) % 3}) {
            const double off = distance_from_line(tris[n][c], p, d);
            if ((tris[n][c] - v).dot(d) > 0 && off < nearest) {
                nearest = off;
                best = std::make_tuple(n, j, c);
            }
        }
    }
    return best;
}

} // namespace

std::optional<Vector3d> carry(const Triangle2& t, const Vector3d& l, const Triangle2& n)
{
    Vector3d out = Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        if (l[k] == 0) continue;
        const int c = corner_of(n, t[k]);
        if (c < 0) return std::nullopt;
        out[c] = l[k];
    }
    return out;
}

std::optional<std::vector<WalkPiece>>
walk_segment(const std::vector<Triangle2>& tris, const WalkPoint& from, const WalkPoint& to)
{
    std::vector<WalkPiece> pieces;
    if (from.tri == to.tri) {
        pieces.push_back({from.tri, {from.bary, to.bary}});
        return pieces;
    }
    Vector2d p = position(tris[from.tri], from.bary);
    const Vector2d q = position(tris[to.tri], to.bary);
    if (p == q) return pieces;
    Vector2d d = q - p;

    size_t t = from.tri;
    Vector3d at = from.bary;
    // A start on an edge is on two triangles: take the one it goes into.
    if (zeros(at) == 1) {
        const int z = at[0] == 0 ? 0 : (at[1] == 0 ? 1 : 2);
        const Vector2d& a = tris[t][(z + 1) % 3];
        const Vector2d& b = tris[t][(z + 2) % 3];
        if (wmtk::utils::predicates::orient2d(a, b, q) ==
            wmtk::utils::predicates::Orientation::NEGATIVE) {
            if (const auto n = across(tris, b, a)) {
                if (const auto l = carry(tris[t], at, tris[*n])) {
                    t = *n;
                    at = *l;
                }
            }
        }
    }

    // A straight walk crosses each triangle once, so more steps than this mean an inverted one.
    for (size_t step = 0; step < 4 * tris.size() + 8; ++step) {
        if (t == to.tri) {
            pieces.push_back({t, {at, to.bary}});
            return pieces;
        }
        // The end is on a corner or an edge this triangle has too.
        if (const auto end = carry(tris[to.tri], to.bary, tris[t])) {
            pieces.push_back({t, {at, *end}});
            return pieces;
        }
        // On a corner, any of the triangles around it may be the one it goes on through.
        if (zeros(at) == 2) {
            const int k = at[0] != 0 ? 0 : (at[1] != 0 ? 1 : 2);
            const Vector2d v = tris[t][k];
            // Straight on to a corner of the last triangle, whatever the rounding.
            if (const int j = corner_of(tris[to.tri], v); j >= 0) {
                t = to.tri;
                at = Vector3d::Zero();
                at[j] = 1;
                continue;
            }
            if (const auto onwards = onwards_from_corner(tris, v, p, q)) {
                t = onwards->first;
                at = Vector3d::Zero();
                at[onwards->second] = 1;
            } else {
                // Along an edge the line just missed, to the corner next to it ahead.
                const auto ahead = nearest_corner_ahead(tris, v, p, q);
                if (!ahead) return std::nullopt;
                const auto [n, j, c] = *ahead;
                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
                a[j] = 1;
                b[c] = 1;
                pieces.push_back({n, {a, b}});
                t = n;
                at = b;
                p = tris[n][c];
                if (p == q) return pieces;
                d = q - p;
                continue;
            }
        }

        const Triangle2& tri = tris[t];
        const std::array<int, 3> s = {{side(p, q, tri[0]), side(p, q, tri[1]), side(p, q, tri[2])}};
        // Leaving through an edge: its start on the right of the line, its end on the left.
        int edge = -1;
        for (int i = 0; i < 3; ++i) {
            if (s[i] < 0 && s[(i + 1) % 3] > 0) edge = i;
        }
        if (edge >= 0) {
            const int a = edge, b = (edge + 1) % 3;
            if (const auto n = across(tris, tri[b], tri[a])) {
                const auto reach = [&](const Vector2d& c) {
                    return std::abs(d.x() * (c.y() - p.y()) - d.y() * (c.x() - p.x()));
                };
                const double ra = reach(tri[a]), rb = reach(tri[b]);
                const double u = ra + rb > 0 ? ra / (ra + rb) : 0.5;
                Vector3d next = Vector3d::Zero();
                next[a] = 1 - u;
                next[b] = u;
                pieces.push_back({t, {at, next}});
                const auto l = carry(tri, next, tris[*n]);
                if (!l) return std::nullopt;
                t = *n;
                at = *l;
                continue;
            }
        }
        // Leaving through a corner: the farthest of the corners on the line.
        int corner = -1;
        for (int k = 0; k < 3; ++k) {
            if (s[k] != 0) continue;
            if (corner < 0 || (tri[k] - tri[corner]).dot(d) > 0) corner = k;
        }
        // Both ends are on the triangulation, so a segment leaves it, or misses a triangle it
        // started on, only by rounding: it goes by the corner it just missed.
        if (corner < 0) {
            double nearest = std::numeric_limits<double>::infinity();
            for (int k = 0; k < 3; ++k) {
                const double off = distance_from_line(tri[k], p, d);
                if ((tri[k] - p).dot(d) > 0 && off < nearest) {
                    nearest = off;
                    corner = k;
                }
            }
            if (corner < 0) return std::nullopt;
            p = tri[corner];
            if (p == q) {
                Vector3d last = Vector3d::Zero();
                last[corner] = 1;
                pieces.push_back({t, {at, last}});
                return pieces;
            }
            d = q - p;
        }
        Vector3d next = Vector3d::Zero();
        next[corner] = 1;
        if (next == at) return std::nullopt;
        pieces.push_back({t, {at, next}});
        at = next;
    }
    return std::nullopt;
}

} // namespace wmtk::components::tracking
