#include <catch2/catch_test_macros.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/ConsistentTopology.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <random>
#include <set>
#include <vector>

using namespace wmtk;
using namespace wmtk::utils;

// Every check here is on connectivity alone, against an answer found from the cells' vertex lists
// without the mesh's own adjacency: whether the region meets the cell in a disk of whole facets,
// and the region's components and Euler characteristic.
namespace {

template <size_t N>
std::array<size_t, N> sorted(std::array<size_t, N> a)
{
    std::sort(a.begin(), a.end());
    return a;
}

/// The closed union of `cells` as a simplicial complex: its simplices by sorted vertices.
template <size_t N>
struct Closure
{
    std::set<size_t> v;
    std::set<std::array<size_t, 2>> e;
    std::set<std::array<size_t, 3>> f;

    void add(const std::array<size_t, N>& c)
    {
        for (size_t i = 0; i < N; ++i) {
            v.insert(c[i]);
            for (size_t j = i + 1; j < N; ++j) {
                e.insert(sorted<2>({{c[i], c[j]}}));
                for (size_t k = j + 1; k < N; ++k) {
                    f.insert(sorted<3>({{c[i], c[j], c[k]}}));
                }
            }
        }
    }
};

/// Components of the closed union (joined through vertices) and its Euler characteristic.
template <size_t N>
std::array<long, 2> signature(const std::vector<std::array<size_t, N>>& cells)
{
    Closure<N> K;
    std::map<size_t, size_t> parent;
    const auto find = [&](size_t x) {
        while (parent[x] != x) {
            x = parent[x] = parent[parent[x]];
        }
        return x;
    };
    for (const auto& c : cells) {
        K.add(c);
        for (const size_t v : c) {
            parent.emplace(v, v);
        }
        for (const size_t v : c) {
            parent[find(v)] = find(c[0]);
        }
    }
    std::set<size_t> roots;
    for (const size_t v : K.v) {
        roots.insert(find(v));
    }
    long euler = long(K.v.size()) - long(K.e.size()) + long(K.f.size());
    if (N == 4) {
        euler -= long(cells.size());
    }
    return {{long(roots.size()), euler}};
}

/// Whether the cells of `region` meet cell c exactly in a union of 1 to N - 1 of its facets.
template <size_t N>
bool meets_in_disk(const std::array<size_t, N>& c, const std::vector<std::array<size_t, N>>& region)
{
    // each simplex of c that a cell of the region holds, as the set of c's corners it uses
    std::set<std::vector<size_t>> met;
    size_t n_facets = 0;
    std::set<size_t> facet_corners;
    std::set<std::vector<size_t>> facet_subsets;
    for (size_t mask = 1; mask < (size_t(1) << N); ++mask) {
        std::vector<size_t> s;
        for (size_t i = 0; i < N; ++i) {
            if (mask >> i & 1) {
                s.push_back(c[i]);
            }
        }
        std::sort(s.begin(), s.end());
        const bool held = std::any_of(region.begin(), region.end(), [&](auto r) {
            std::sort(r.begin(), r.end());
            return std::includes(r.begin(), r.end(), s.begin(), s.end());
        });
        if (!held) {
            continue;
        }
        met.insert(s);
        if (s.size() == N - 1) {
            ++n_facets;
            for (size_t sub = 1; sub < (size_t(1) << s.size()); ++sub) {
                std::vector<size_t> t;
                for (size_t i = 0; i < s.size(); ++i) {
                    if (sub >> i & 1) {
                        t.push_back(s[i]);
                    }
                }
                facet_subsets.insert(t);
            }
        }
    }
    return n_facets >= 1 && n_facets <= N - 1 && met == facet_subsets;
}

/// nx x ny squares, each cut in two along a diagonal chosen by `rng`.
std::vector<std::array<size_t, 3>> grid_tris(const size_t nx, const size_t ny, std::mt19937& rng)
{
    const auto id = [&](const size_t i, const size_t j) { return i + j * (nx + 1); };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            const size_t a = id(i, j), b = id(i + 1, j), c = id(i + 1, j + 1), d = id(i, j + 1);
            if (rng() & 1) {
                tris.push_back({{a, b, c}});
                tris.push_back({{a, c, d}});
            } else {
                tris.push_back({{a, b, d}});
                tris.push_back({{b, c, d}});
            }
        }
    }
    return tris;
}

/// nx x ny x nz cubes, each cut into six tets around its main diagonal.
std::vector<std::array<size_t, 4>> grid_tets(const size_t nx, const size_t ny, const size_t nz)
{
    const auto id = [&](const size_t i, const size_t j, const size_t k) {
        return i + (nx + 1) * (j + (ny + 1) * k);
    };
    std::vector<std::array<size_t, 4>> tets;
    for (size_t k = 0; k < nz; ++k) {
        for (size_t j = 0; j < ny; ++j) {
            for (size_t i = 0; i < nx; ++i) {
                const size_t p0 = id(i, j, k), p7 = id(i + 1, j + 1, k + 1);
                const size_t px = id(i + 1, j, k), py = id(i, j + 1, k), pz = id(i, j, k + 1);
                const size_t pxy = id(i + 1, j + 1, k), pxz = id(i + 1, j, k + 1),
                             pyz = id(i, j + 1, k + 1);
                tets.push_back({{p0, px, pxy, p7}});
                tets.push_back({{p0, pxy, py, p7}});
                tets.push_back({{p0, py, pyz, p7}});
                tets.push_back({{p0, pyz, pz, p7}});
                tets.push_back({{p0, pz, pxz, p7}});
                tets.push_back({{p0, pxz, px, p7}});
            }
        }
    }
    return tets;
}

/// Random regions on the mesh: the test against the disk rule at every cell off the region, and
/// the topology kept each time the test says yes. Returns how many cells passed.
template <size_t N, typename Mesh, typename Test>
size_t check_random_regions(
    const Mesh& m,
    const std::vector<std::array<size_t, N>>& cells,
    const Test& test,
    std::mt19937& rng,
    const int n_regions)
{
    size_t n_passed = 0;
    for (int trial = 0; trial < n_regions; ++trial) {
        // from thin to dense, so every number of shared facets turns up
        std::uniform_real_distribution<double> uni(0., 1.);
        const double density = 0.1 + 0.8 * uni(rng);
        std::vector<char> in(cells.size(), 0);
        std::vector<std::array<size_t, N>> region;
        for (size_t c = 0; c < cells.size(); ++c) {
            in[c] = uni(rng) < density;
            if (in[c]) {
                region.push_back(cells[c]);
            }
        }
        const std::array<long, 2> before = signature<N>(region);
        for (size_t c = 0; c < cells.size(); ++c) {
            if (in[c]) {
                continue;
            }
            const bool consistent = test(m, c, [&](const size_t x) {
                REQUIRE(x != c);
                return in[x] != 0;
            });
            CHECK(consistent == meets_in_disk<N>(cells[c], region));
            if (consistent) {
                ++n_passed;
                std::vector<std::array<size_t, N>> grown = region;
                grown.push_back(cells[c]);
                CHECK(signature<N>(grown) == before);
            }
        }
    }
    return n_passed;
}

} // namespace

TEST_CASE("consistent_topology_tri_cases", "[consistent_topology][2d]")
{
    // a fan of six triangles around vertex 0, closed; face 0 is 0-1-2, face i is 0-(i+1)-(i+2)
    TriMesh m;
    m.init(7, {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}, {{0, 4, 5}}, {{0, 5, 6}}, {{0, 6, 1}}});
    const auto region = [](const std::set<size_t>& X) {
        return [X](const size_t f) { return X.count(f) > 0; };
    };
    CHECK_FALSE(tri_consistent_topology(m, 0, region({}))); // a new piece
    CHECK_FALSE(tri_consistent_topology(m, 0, region({3}))); // a corner shared, no edge
    CHECK(tri_consistent_topology(m, 0, region({1}))); // one edge, third corner free
    CHECK(tri_consistent_topology(m, 0, region({1, 2}))); // the same, the rest of the fan beyond
    CHECK(tri_consistent_topology(m, 0, region({1, 3}))); // face 3 meets the cell in X's corner 0
    CHECK(tri_consistent_topology(m, 0, region({1, 5}))); // two edges, around vertex 0
    CHECK(tri_consistent_topology(m, 0, region({1, 2, 3, 4, 5}))); // closes the fan at a disk
}

TEST_CASE("consistent_topology_tri_tunnel_and_hole", "[consistent_topology][2d]")
{
    // a face with a neighbour on each edge; the region reaches its third corner round the back
    TriMesh m;
    m.init(
        7,
        {{{0, 1, 2}}, // 0: the cell
         {{1, 0, 3}}, // 1: across 0-1
         {{2, 1, 4}}, // 2: across 1-2
         {{0, 2, 5}}, // 3: across 2-0
         {{2, 5, 6}}}); // 4: holds corner 2, shares no edge with the cell
    const auto region = [](const std::set<size_t>& X) {
        return [X](const size_t f) { return X.count(f) > 0; };
    };
    CHECK(tri_consistent_topology(m, 0, region({1})));
    // one edge, and corner 2 touches the region elsewhere: adding the cell closes a loop
    CHECK_FALSE(tri_consistent_topology(m, 0, region({1, 4})));
    // two edges: 0-1 and 1-2
    CHECK(tri_consistent_topology(m, 0, region({1, 2})));
    // all three edges: the cell is a hole in the region
    CHECK_FALSE(tri_consistent_topology(m, 0, region({1, 2, 3})));
}

TEST_CASE("consistent_topology_tet_cases", "[consistent_topology][3d]")
{
    // tet 0 = 0-1-2-3 with a neighbour across each face, and tets on its far corners and edge
    TetMesh m;
    m.init(
        10,
        {{{0, 1, 2, 3}}, // 0: the cell
         {{1, 0, 2, 4}}, // 1: across 0-1-2, corner 3 opposite
         {{0, 1, 3, 5}}, // 2: across 0-1-3, corner 2 opposite
         {{0, 2, 3, 6}}, // 3: across 0-2-3, corner 1 opposite
         {{1, 2, 3, 7}}, // 4: across 1-2-3, corner 0 opposite
         {{3, 5, 6, 8}}, // 5: holds corner 3 alone
         {{2, 3, 8, 9}}}); // 6: holds the edge 2-3, on neither face 0-1-2 nor 0-1-3
    const auto region = [](const std::set<size_t>& X) {
        return [X](const size_t t) { return X.count(t) > 0; };
    };
    CHECK_FALSE(tet_consistent_topology(m, 0, region({}))); // a new piece
    CHECK_FALSE(tet_consistent_topology(m, 0, region({5}))); // a corner only
    CHECK_FALSE(tet_consistent_topology(m, 0, region({6}))); // an edge only
    CHECK(tet_consistent_topology(m, 0, region({1}))); // one face, corner 3 free
    // one face, and the opposite corner touches: a tunnel
    CHECK_FALSE(tet_consistent_topology(m, 0, region({1, 5})));
    CHECK(tet_consistent_topology(m, 0, region({1, 2}))); // two faces, edge 2-3 free
    // two faces, and the edge on neither touches: a tunnel
    CHECK_FALSE(tet_consistent_topology(m, 0, region({1, 2, 6})));
    // the corners of that edge alone do not count against it: both are on the two faces
    CHECK(tet_consistent_topology(m, 0, region({1, 2, 5})));
    CHECK(tet_consistent_topology(m, 0, region({1, 2, 3}))); // three faces: always a disk
    CHECK(tet_consistent_topology(m, 0, region({1, 2, 3, 5, 6})));
    CHECK_FALSE(tet_consistent_topology(m, 0, region({1, 2, 3, 4}))); // all four: a void
}

TEST_CASE("consistent_topology_tri_random", "[consistent_topology][2d]")
{
    std::mt19937 rng(1);
    const std::vector<std::array<size_t, 3>> tris = grid_tris(5, 5, rng);
    TriMesh m;
    m.init(36, tris);
    const size_t n_passed = check_random_regions<3>(m, tris, tri_consistent_topology, rng, 40);
    CHECK(n_passed > 0);
}

TEST_CASE("consistent_topology_tet_random", "[consistent_topology][3d]")
{
    std::mt19937 rng(2);
    const std::vector<std::array<size_t, 4>> tets = grid_tets(2, 2, 2);
    TetMesh m;
    m.init(27, tets);
    const size_t n_passed = check_random_regions<4>(m, tets, tet_consistent_topology, rng, 40);
    CHECK(n_passed > 0);
}
