#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
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

Patch make_patch(int rows, int cols)
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
