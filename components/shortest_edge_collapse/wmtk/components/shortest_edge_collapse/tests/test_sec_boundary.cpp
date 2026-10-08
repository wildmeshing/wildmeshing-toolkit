#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

using namespace wmtk;
using namespace components::shortest_edge_collapse;

namespace {

/// A flat N x N triangulated patch. It is an open surface, so freeze_boundary() freezes its
/// whole outline -- 4*(N-1) of the N*N vertices.
struct Patch
{
    std::vector<Eigen::Vector3d> v;
    std::vector<std::array<size_t, 3>> f;
    std::set<std::pair<long long, long long>> boundary_positions; // quantised, for lookup
};

/// `hole` removes the two triangles of grid cell (hole, hole), opening a square hole of side
/// 1/(rows-1) (for a square patch); -1 for none. boundary_positions lists the OUTER outline.
Patch make_patch(int rows, int cols, int hole = -1)
{
    Patch p;
    const auto id = [cols](int i, int j) { return size_t(i * cols + j); };
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            p.v.emplace_back(double(i) / (rows - 1), double(j) / (cols - 1), 0.0);
        }
    }
    for (int i = 0; i + 1 < rows; ++i) {
        for (int j = 0; j + 1 < cols; ++j) {
            if (i == hole && j == hole) continue;
            p.f.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            p.f.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (i == 0 || j == 0 || i == rows - 1 || j == cols - 1) {
                const Eigen::Vector3d& q = p.v[id(i, j)];
                p.boundary_positions.insert(
                    {(long long)std::llround(q[0] * 1e9), (long long)std::llround(q[1] * 1e9)});
            }
        }
    }
    return p;
}

/// Collapse `p` as far as it goes and check the frozen outline came through untouched.
size_t collapse_and_check(const Patch& p)
{
    ShortestEdgeCollapse m(p.v, 0, false);
    m.create_mesh(p.v.size(), p.f, {}, 1e-3);
    m.collapse_shortest(0);
    m.consolidate_mesh();

    std::set<std::pair<long long, long long>> after;
    for (const auto& t : m.get_vertices()) {
        const Eigen::Vector3d& q = m.vertex_attrs[t.vid(m)].pos;
        after.insert({(long long)std::llround(q[0] * 1e9), (long long)std::llround(q[1] * 1e9)});
    }
    // Every original outline position must still be there, exactly.
    for (const auto& b : p.boundary_positions) {
        CHECK(after.count(b) == 1);
    }
    return m.get_vertices().size();
}

double dist_to_segment(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const Eigen::Vector3d ab = b - a;
    const double t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

/// Distance to the outline of the axis-aligned square [lo, hi]^2 in the z = 0 plane.
double dist_to_square(const Eigen::Vector3d& p, double lo, double hi)
{
    const Eigen::Vector3d c[4] = {{lo, lo, 0}, {hi, lo, 0}, {hi, hi, 0}, {lo, hi, 0}};
    double d = std::numeric_limits<double>::max();
    for (int k = 0; k < 4; ++k) d = std::min(d, dist_to_segment(p, c[k], c[(k + 1) % 4]));
    return d;
}

/// Farthest any point of segment ab gets from a set, by its distance function (sampled).
template <typename Dist>
double max_dist_on_segment(const Eigen::Vector3d& a, const Eigen::Vector3d& b, Dist dist)
{
    double d = 0;
    for (int k = 0; k <= 32; ++k) d = std::max(d, dist(a + (b - a) * (k / 32.0)));
    return d;
}

std::vector<std::array<Eigen::Vector3d, 2>> boundary_edges(ShortestEdgeCollapse& m)
{
    std::vector<std::array<Eigen::Vector3d, 2>> out;
    for (const auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            out.push_back(
                {{m.vertex_attrs[e.vid(m)].pos, m.vertex_attrs[e.switch_vertex(m).vid(m)].pos}});
        }
    }
    return out;
}

/// Simplify inside a surface envelope of eps, with a boundary tube of radius r instead of a
/// frozen boundary.
void collapse_with_tube(
    ShortestEdgeCollapse& m,
    const Patch& p,
    double eps,
    double r,
    bool link_condition)
{
    m.set_use_link_condition(link_condition);
    m.create_mesh(p.v.size(), p.f, {}, eps, r);
    m.collapse_shortest(0);
    m.consolidate_mesh();
    REQUIRE(m.check_mesh_connectivity_validity());
}

} // namespace

TEST_CASE("sec-open-surface-boundary-is-preserved", "[test_sec][boundary]")
{
    // A collapse must never move the frozen outline of an open surface. The interior,
    // however, must coarsen all the way down -- including the vertices that merely touch
    // the outline, which collapse *onto* it. The outline itself is the floor: an edge with
    // two frozen endpoints stays.
    const int n = 21;
    const Patch p = make_patch(n, n);
    const size_t n_boundary = p.boundary_positions.size();
    REQUIRE(n_boundary == size_t(4 * (n - 1)));

    const size_t n_after = collapse_and_check(p);
    logger().info(
        "[sec-boundary] square patch: {} vertices ({} on the frozen outline) -> {}",
        p.v.size(),
        n_boundary,
        n_after);

    CHECK(n_after >= n_boundary);
    // Nothing but the outline should be left: every interior vertex can reach it.
    CHECK(n_after == n_boundary);
}

TEST_CASE("sec-open-strip-boundary-is-preserved", "[test_sec][boundary]")
{
    // A strip is the adversarial case for the freeze rule: almost every vertex touches the
    // outline, so rejecting any collapse with a frozen endpoint would leave nearly the whole
    // mesh un-collapsible.
    const Patch p = make_patch(3, 60);
    const size_t n_boundary = p.boundary_positions.size();

    const size_t n_after = collapse_and_check(p);
    logger().info(
        "[sec-boundary] strip: {} vertices ({} on the frozen outline) -> {}",
        p.v.size(),
        n_boundary,
        n_after);

    CHECK(n_after >= n_boundary);
    CHECK(n_after == n_boundary);
}

TEST_CASE("sec-boundary-envelope-outline-stays-in-tube", "[test_sec][boundary]")
{
    // With a boundary tube the outline is free to coarsen, but every boundary edge has to stay
    // within r of the input outline -- and since the outline is a closed loop that can only
    // change by chords the tube admits, it cannot retract either: a corner is cut by at most
    // the chord whose midpoint touches the tube, which for a right angle leaves the corner
    // sqrt(2) * r away.
    const int n = 21;
    const Patch p = make_patch(n, n);
    const double r = 0.01;
    ShortestEdgeCollapse m(p.v, 0, false);
    collapse_with_tube(m, p, 1e-3, r, true);

    const auto be = boundary_edges(m);
    REQUIRE(!be.empty());
    const auto outline = [](const Eigen::Vector3d& q) { return dist_to_square(q, 0, 1); };
    for (const auto& e : be) {
        CHECK(max_dist_on_segment(e[0], e[1], outline) <= r);
    }
    for (const Eigen::Vector3d corner :
         {Eigen::Vector3d(0, 0, 0),
          Eigen::Vector3d(1, 0, 0),
          Eigen::Vector3d(1, 1, 0),
          Eigen::Vector3d(0, 1, 0)}) {
        double d = std::numeric_limits<double>::max();
        for (const auto& e : be) d = std::min(d, dist_to_segment(corner, e[0], e[1]));
        CHECK(d <= std::sqrt(2.0) * r + 1e-12);
    }

    const size_t n_after = m.get_vertices().size();
    logger().info(
        "[sec-boundary] square patch with a boundary tube: {} vertices -> {} ({} boundary edges)",
        p.v.size(),
        n_after,
        be.size());
    // Frozen, the outline alone keeps 4*(n-1) = 80 vertices. Free, its straight sides coarsen.
    CHECK(n_after < size_t(4 * (n - 1)) / 2);
}

TEST_CASE("sec-boundary-envelope-closes-hole-smaller-than-tube", "[test_sec][boundary]")
{
    // A 0.05-wide hole in the middle of the patch, and a tube wider than the hole: closing it
    // keeps every boundary edge in the tube, so the hole goes. The link condition is off, as
    // tetwild runs the simplification -- with it on, the last collapse that closes a hole is
    // exactly the one it refuses. The surface envelope has to admit the triangles that cover
    // the hole too; it is wide here so that the tube is what decides, as it is in tetwild,
    // whose simplification tube is half its surface envelope.
    const int n = 21;
    const Patch p = make_patch(n, n, n / 2);
    const double r = 0.06;
    ShortestEdgeCollapse m(p.v, 0, false);
    collapse_with_tube(m, p, 0.1, r, false);

    const auto outline = [](const Eigen::Vector3d& q) { return dist_to_square(q, 0, 1); };
    size_t inner = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], outline) > r) ++inner;
    }
    CHECK(inner == 0);
}

TEST_CASE("sec-boundary-envelope-keeps-hole-larger-than-tube", "[test_sec][boundary]")
{
    // The same hole against a tube a fifth of its width, with the same wide surface envelope:
    // the tube alone keeps it from closing, and its outline stays within the tube.
    const int n = 21;
    const int h = n / 2;
    const Patch p = make_patch(n, n, h);
    const double r = 0.01;
    ShortestEdgeCollapse m(p.v, 0, false);
    collapse_with_tube(m, p, 0.1, r, false);

    const double lo = double(h) / (n - 1), hi = double(h + 1) / (n - 1);
    const auto outline = [](const Eigen::Vector3d& q) { return dist_to_square(q, 0, 1); };
    const auto hole = [&](const Eigen::Vector3d& q) { return dist_to_square(q, lo, hi); };
    size_t inner = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], outline) <= r) continue;
        ++inner;
        CHECK(max_dist_on_segment(e[0], e[1], hole) <= r);
    }
    CHECK(inner >= 3);
}
