#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
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
using namespace components::isotropic_remeshing;

// The boundary tube on isotropic remeshing, mirroring shortest_edge_collapse's
// test_sec_boundary.cpp. Unlike a simplification, a remeshing pass splits, swaps and smooths as
// well as collapsing, and the tube has to hold through all four.

namespace {

/// A flat rows x cols patch on [0, 1]^2. `hole` removes the two triangles of grid cell
/// (hole, hole), opening a square hole of side 1/(rows-1); -1 for none.
struct Patch
{
    std::vector<Eigen::Vector3d> v;
    std::vector<std::array<size_t, 3>> f;
};

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
    return p;
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

std::vector<std::array<Eigen::Vector3d, 2>> boundary_edges(IsotropicRemeshing& m)
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

/// Remesh towards edge length L inside a surface envelope of eps and a boundary tube of radius
/// r (0: none, the default). freeze_boundary is passed as true throughout, the application's
/// default, so the tube tests also check that the tube replaces the freezing.
void remesh(
    IsotropicRemeshing& m,
    const Patch& p,
    double eps,
    double r,
    bool link_condition,
    double L,
    int iterations)
{
    m.set_use_link_condition(link_condition);
    m.create_mesh(p.v.size(), p.f, {}, true, eps, r);
    REQUIRE(m.check_mesh_connectivity_validity());
    m.uniform_remeshing(L, iterations);
    m.consolidate_mesh();
    REQUIRE(m.check_mesh_connectivity_validity());
}

const auto outline = [](const Eigen::Vector3d& q) { return dist_to_square(q, 0, 1); };

} // namespace

TEST_CASE("remeshing-boundary-envelope-outline-stays-in-tube", "[test_remeshing][boundary]")
{
    // Coarsening a 0.05 grid towards L = 0.25: the outline coarsens with the rest, every boundary
    // edge stays within r of the input outline, and the corners -- which smoothing would round
    // off and the collapses' midpoints cut -- stay covered within sqrt(2) * r. No vertex is
    // frozen although freeze_boundary is on.
    const int n = 21;
    const Patch p = make_patch(n, n);
    const double r = 0.01;
    // Sampled predicates: with the exact ones, the surface envelope this thin takes ~10 s here.
    // The other tests use the exact ones, the application's default.
    IsotropicRemeshing m(p.v, 0, false);
    remesh(m, p, 1e-3, r, true, 0.25, 3);

    for (const auto& v : m.get_vertices()) {
        CHECK(!m.vertex_attrs[v.vid(m)].freeze);
    }
    const auto be = boundary_edges(m);
    REQUIRE(!be.empty());
    for (const auto& e : be) {
        CHECK(max_dist_on_segment(e[0], e[1], outline) <= r);
    }
    for (const Eigen::Vector3d& corner :
         {Eigen::Vector3d(0, 0, 0),
          Eigen::Vector3d(1, 0, 0),
          Eigen::Vector3d(1, 1, 0),
          Eigen::Vector3d(0, 1, 0)}) {
        double d = std::numeric_limits<double>::max();
        for (const auto& e : be) d = std::min(d, dist_to_segment(corner, e[0], e[1]));
        CHECK(d <= std::sqrt(2.0) * r + 1e-12);
    }
    logger().info(
        "[remeshing-boundary] square patch with a boundary tube: {} vertices -> {} ({} boundary "
        "edges)",
        p.v.size(),
        m.get_vertices().size(),
        be.size());
    // Frozen, the outline would keep all 4 * (n - 1) = 80 of its edges.
    CHECK(be.size() < size_t(4 * (n - 1)) / 2);
}

TEST_CASE("remeshing-boundary-envelope-split-carries-lineage", "[test_remeshing][boundary]")
{
    // Refining a 0.2 grid towards L = 0.05 splits every boundary edge several times over. Each
    // new boundary vertex has to inherit input_boundary -- otherwise the halves between it and
    // its unflagged siblings would escape the tube -- and smoothing then moves them along the
    // outline without leaving it.
    const int n = 6;
    const Patch p = make_patch(n, n);
    const double r = 0.01;
    IsotropicRemeshing m(p.v, 0);
    remesh(m, p, 1e-3, r, true, 0.05, 2);

    size_t n_boundary = 0;
    for (const auto& v : m.get_vertices()) {
        if (!m.is_boundary_vertex(v)) continue;
        ++n_boundary;
        CHECK(m.vertex_attrs[v.vid(m)].input_boundary);
    }
    for (const auto& e : boundary_edges(m)) {
        CHECK(max_dist_on_segment(e[0], e[1], outline) <= r);
    }
    // Refined: 4 * (n - 1) = 20 on the input.
    CHECK(n_boundary > size_t(4 * (n - 1)) * 2);
}

TEST_CASE("remeshing-boundary-envelope-closes-hole-smaller-than-tube", "[test_remeshing][boundary]")
{
    // A 0.05-wide hole and a tube wider than it: closing the hole keeps every boundary edge in
    // the tube, so it goes -- with the link condition off, which refuses exactly the collapse
    // that closes a hole, and inside a surface envelope wide enough to admit the triangles that
    // cover it.
    const int n = 21;
    const Patch p = make_patch(n, n, n / 2);
    const double r = 0.06;
    IsotropicRemeshing m(p.v, 0);
    remesh(m, p, 0.1, r, false, 0.25, 3);

    size_t inner = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], outline) > r) ++inner;
    }
    CHECK(inner == 0);
}

TEST_CASE("remeshing-boundary-envelope-keeps-hole-larger-than-tube", "[test_remeshing][boundary]")
{
    // The same hole against a tube a fifth of its width, with the same wide surface envelope and
    // the link condition still off: the tube alone keeps it open, and its outline in the tube.
    const int n = 21;
    const int h = n / 2;
    const Patch p = make_patch(n, n, h);
    const double r = 0.01;
    IsotropicRemeshing m(p.v, 0);
    remesh(m, p, 0.1, r, false, 0.25, 3);

    const double lo = double(h) / (n - 1), hi = double(h + 1) / (n - 1);
    const auto hole = [&](const Eigen::Vector3d& q) { return dist_to_square(q, lo, hi); };
    size_t inner = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], outline) <= r) continue;
        ++inner;
        CHECK(max_dist_on_segment(e[0], e[1], hole) <= r);
    }
    CHECK(inner >= 3);
}

TEST_CASE("remeshing-default-boundary-is-frozen", "[test_remeshing][boundary]")
{
    // Without a tube freeze_boundary freezes the input's boundary, as it always has: every
    // outline and hole vertex comes through exactly where it was, and nothing is flagged.
    const int n = 21;
    const int h = n / 2;
    const Patch p = make_patch(n, n, h);
    IsotropicRemeshing m(p.v, 0);
    remesh(m, p, 0.1, 0, false, 0.25, 3);

    CHECK(!m.m_boundary_envelope.initialized());
    const auto key = [](const Eigen::Vector3d& q) {
        return std::make_pair(std::llround(q[0] * 1e9), std::llround(q[1] * 1e9));
    };
    std::set<std::pair<long long, long long>> after;
    for (const auto& v : m.get_vertices()) {
        CHECK(!m.vertex_attrs[v.vid(m)].input_boundary);
        after.insert(key(m.vertex_attrs[v.vid(m)].pos));
    }
    const double lo = double(h) / (n - 1), hi = double(h + 1) / (n - 1);
    size_t n_input_boundary = 0;
    for (const Eigen::Vector3d& q : p.v) {
        if (outline(q) > 1e-12 && dist_to_square(q, lo, hi) > 1e-12) continue;
        ++n_input_boundary;
        CHECK(after.count(key(q)) == 1);
    }
    CHECK(n_input_boundary == size_t(4 * (n - 1) + 4));
}
