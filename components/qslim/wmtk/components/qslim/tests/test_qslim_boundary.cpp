#include <wmtk/components/qslim/QSlimMesh.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

using namespace wmtk;
using namespace components::qslim;

// The boundary tube on qslim, mirroring shortest_edge_collapse's test_sec_boundary.cpp. On a flat
// patch every quadric is rank one, so qslim places each collapse on one of its endpoints: the
// placement is not what these tests are about, the tube is.

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

std::vector<std::array<Eigen::Vector3d, 2>> boundary_edges(QSlimMesh& m)
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

/// Collapse as far as qslim goes, inside a surface envelope of eps and a boundary tube of
/// radius r (0: none, the default).
void collapse(QSlimMesh& m, const Patch& p, double eps, double r, bool link_condition)
{
    m.set_use_link_condition(link_condition);
    m.create_mesh(p.v.size(), p.f, {}, eps, r);
    REQUIRE(m.check_mesh_connectivity_validity());
    m.collapse_qslim(0);
    m.consolidate_mesh();
    REQUIRE(m.check_mesh_connectivity_validity());
}

const auto outline = [](const Eigen::Vector3d& q) { return dist_to_square(q, 0, 1); };

} // namespace

TEST_CASE("qslim-boundary-envelope-outline-stays-in-tube", "[test_qslim][boundary]")
{
    // Every boundary edge stays within r of the input outline, which, the outline being a closed
    // loop that can only change by chords the tube admits, also keeps it from retracting: a
    // corner is cut by at most the chord whose midpoint touches the tube, sqrt(2) * r away.
    const int n = 21;
    const Patch p = make_patch(n, n);
    const double r = 0.01;
    QSlimMesh m(p.v, 0);
    collapse(m, p, 1e-3, r, true);

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

    const size_t n_after = m.get_vertices().size();
    logger().info(
        "[qslim-boundary] square patch with a boundary tube: {} vertices -> {} ({} boundary edges)",
        p.v.size(),
        n_after,
        be.size());
    // The outline is not frozen: its straight sides coarsen to a handful of edges.
    CHECK(n_after < size_t(4 * (n - 1)) / 2);
}

TEST_CASE("qslim-boundary-envelope-curved-outline-coarsens", "[test_qslim][boundary]")
{
    // On a curved surface the quadric's free optimum for an edge along the boundary lies off
    // the outline, and the tube refuses it: unless the collapse is placed on the edge, the
    // outline of this cap keeps every one of its 120 edges. Placed on the edge, it coarsens.
    const int nlat = 20, nlon = 120;
    const double pi = std::acos(-1.0);
    const double th0 = 110.0 * pi / 180.0;
    Patch p;
    p.v.emplace_back(0, 0, 1);
    for (int i = 1; i <= nlat; ++i) {
        const double th = th0 * i / nlat;
        for (int j = 0; j < nlon; ++j) {
            const double ph = 2 * pi * j / nlon;
            p.v.emplace_back(
                std::sin(th) * std::cos(ph),
                std::sin(th) * std::sin(ph),
                std::cos(th));
        }
    }
    const auto ring = [](int i, int j) { return size_t(1 + (i - 1) * nlon + (j % nlon)); };
    for (int j = 0; j < nlon; ++j) p.f.push_back({{0, ring(1, j), ring(1, j + 1)}});
    for (int i = 1; i < nlat; ++i) {
        for (int j = 0; j < nlon; ++j) {
            p.f.push_back({{ring(i, j), ring(i + 1, j), ring(i + 1, j + 1)}});
            p.f.push_back({{ring(i, j), ring(i + 1, j + 1), ring(i, j + 1)}});
        }
    }
    std::vector<std::array<Eigen::Vector3d, 2>> rim;
    for (int j = 0; j < nlon; ++j) rim.push_back({{p.v[ring(nlat, j)], p.v[ring(nlat, j + 1)]}});
    const auto to_rim = [&](const Eigen::Vector3d& q) {
        double d = std::numeric_limits<double>::max();
        for (const auto& e : rim) d = std::min(d, dist_to_segment(q, e[0], e[1]));
        return d;
    };

    Eigen::Vector3d lo = p.v[0], hi = p.v[0];
    for (const auto& q : p.v) {
        lo = lo.cwiseMin(q);
        hi = hi.cwiseMax(q);
    }
    const double eps = 1e-3 * (hi - lo).norm();
    const double r = 2 * eps;
    QSlimMesh m(p.v, 0);
    m.create_mesh(p.v.size(), p.f, {}, eps, r);
    m.collapse_qslim(int(p.v.size() / 10));
    m.consolidate_mesh();
    REQUIRE(m.check_mesh_connectivity_validity());

    const auto be = boundary_edges(m);
    for (const auto& e : be) {
        CHECK(max_dist_on_segment(e[0], e[1], to_rim) <= r);
    }
    logger().info(
        "[qslim-boundary] cap with a boundary tube: {} vertices -> {}, rim {} edges -> {}",
        p.v.size(),
        m.get_vertices().size(),
        rim.size(),
        be.size());
    CHECK(be.size() < rim.size() * 9 / 10);
}

TEST_CASE("qslim-boundary-envelope-closes-hole-smaller-than-tube", "[test_qslim][boundary]")
{
    // A 0.05-wide hole and a tube wider than it: closing the hole keeps every boundary edge in
    // the tube, so it goes. Only with the link condition off -- the collapse that closes a hole is
    // exactly the one it refuses -- and inside a surface envelope wide enough to admit the
    // triangles that cover the hole, so that the tube is what decides.
    const int n = 21;
    const Patch p = make_patch(n, n, n / 2);
    const double r = 0.06;
    QSlimMesh m(p.v, 0);
    collapse(m, p, 0.1, r, false);

    size_t inner = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], outline) > r) ++inner;
    }
    CHECK(inner == 0);
}

TEST_CASE("qslim-boundary-envelope-keeps-hole-larger-than-tube", "[test_qslim][boundary]")
{
    // The same hole against a tube a fifth of its width, with the same wide surface envelope and
    // the link condition still off: the tube alone keeps it open, and its outline in the tube.
    const int n = 21;
    const int h = n / 2;
    const Patch p = make_patch(n, n, h);
    const double r = 0.01;
    QSlimMesh m(p.v, 0);
    collapse(m, p, 0.1, r, false);

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

TEST_CASE("qslim-default-boundary-is-unchanged", "[test_qslim][boundary]")
{
    // Without a tube qslim holds the boundary by nothing but the surface envelope, as it always
    // has: no vertex carries the lineage flag, and the hole the tube kept open above closes
    // under the same envelope and the same link condition.
    const int n = 21;
    const int h = n / 2;
    const Patch p = make_patch(n, n, h);
    QSlimMesh m(p.v, 0);
    collapse(m, p, 0.1, 0, false);

    CHECK(!m.m_boundary_envelope.initialized());
    for (const auto& v : m.get_vertices()) {
        CHECK(!m.vertex_attrs[v.vid(m)].input_boundary);
    }
    const double lo = double(h) / (n - 1), hi = double(h + 1) / (n - 1);
    const auto hole = [&](const Eigen::Vector3d& q) { return dist_to_square(q, lo, hi); };
    size_t on_hole = 0;
    for (const auto& e : boundary_edges(m)) {
        if (max_dist_on_segment(e[0], e[1], hole) <= 1e-12) ++on_hole;
    }
    CHECK(on_hole == 0);
}
