#pragma once

#include <wmtk/simplex/Simplex.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wmtk::utils {

// Dimension-generic pieces of the "stuck element" sizing-field refinement shared by the
// wild-meshing applications (tetwild, simwild, triwild). Each application keeps its own
// refine_sizing_around_worst() driver -- they differ in which attributes hold the quality
// and the sizing scalar, and simwild additionally caps the sizing by its per-tag sizing
// field -- but the actual algorithms below are identical between them, so they live here.
//
// Everything is expressed through small callables rather than a mesh interface: the data
// these need (quality, sizing scalar, positions) lives in the application's attribute
// collections, not in TetMesh/TriMesh. They are templates, not std::function, because the
// one-ring accessor is called once per vertex inside a BFS.

/// (quality, cell id) of one cell picked for refinement. "Cell" is the top-dimensional
/// element: a tet for tetwild/simwild, a triangle for triwild.
using WorstCell = std::pair<double, size_t>;

/**
 * @brief The `num_worst` valid cells with the highest quality, among those whose energy
 * reaches `filter_energy`.
 *
 * @param n_cells        cell capacity (ids 0..n_cells-1 are probed)
 * @param is_valid       `bool(size_t cid)` -- whether the cell is live
 * @param energy         `double(size_t cid)` -- energy of a cell
 * @param filter_energy  cells below this energy are never refined
 * @param num_worst      how many to keep; <= 0 keeps every cell above filter_energy
 *
 * @return the selected cells sorted ascending by quality (so back() is the worst).
 */
template <class IsValid, class Energy>
std::vector<WorstCell> select_worst_cells(
    size_t n_cells,
    IsValid is_valid,
    Energy energy,
    double filter_energy,
    int num_worst)
{
    // `worst` is kept sorted ascending by quality, size <= num_worst (front = smallest kept).
    std::vector<WorstCell> worst;
    if (num_worst > 0) {
        worst.reserve(num_worst);
    }

    for (size_t cid = 0; cid < n_cells; ++cid) {
        if (!is_valid(cid)) {
            continue;
        }
        const double q = energy(cid);
        if (q < filter_energy) {
            continue;
        }
        if (num_worst > 0) {
            if (static_cast<int>(worst.size()) < num_worst) {
                worst.emplace_back(q, cid);
                std::sort(worst.begin(), worst.end());
            } else if (q > worst.front().first) {
                worst.front() = {q, cid};
                std::sort(worst.begin(), worst.end());
            }
        } else {
            worst.emplace_back(q, cid);
        }
    }

    if (num_worst <= 0) {
        std::sort(worst.begin(), worst.end());
    }
    return worst;
}

/**
 * @brief The longest edge of a cell, as a sorted vertex pair.
 *
 * The force-split mechanism queues exactly these edges so that a stuck sliver's long edge
 * is split immediately, bypassing the sizing-length gate and without touching the sizing
 * field.
 *
 * @param vs  the cell's vertex ids (4 for a tet, 3 for a triangle)
 * @param pos `const Vector&(size_t vid)` -- vertex position, any Eigen vector type
 */
template <size_t N, class Pos>
simplex::Edge longest_edge(const std::array<size_t, N>& vs, Pos pos)
{
    static_assert(N >= 2, "a cell needs at least two vertices to have an edge");
    double l2max = -1;
    size_t ea = vs[0];
    size_t eb = vs[1];
    for (size_t a = 0; a < N; ++a) {
        for (size_t b = a + 1; b < N; ++b) {
            const double l2 = (pos(vs[a]) - pos(vs[b])).squaredNorm();
            if (l2 > l2max) {
                l2max = l2;
                ea = vs[a];
                eb = vs[b];
            }
        }
    }
    return simplex::Edge(ea, eb);
}

/**
 * @brief `seeds` grown by `n_rings` hops along the vertex adjacency.
 *
 * @param one_ring `Range(size_t vid)` -- the vids adjacent to vid (duplicates are fine,
 *                 the region set absorbs them)
 */
template <class OneRing>
std::unordered_set<size_t>
grow_vertex_region(const std::vector<size_t>& seeds, int n_rings, OneRing one_ring)
{
    std::unordered_set<size_t> region(seeds.begin(), seeds.end());
    std::vector<size_t> frontier(region.begin(), region.end());

    for (int r = 0; r < n_rings; ++r) {
        std::vector<size_t> next;
        for (const size_t v : frontier) {
            for (const size_t u : one_ring(v)) {
                if (region.insert(u).second) {
                    next.push_back(u);
                }
            }
        }
        frontier.swap(next);
    }
    return region;
}

/**
 * @brief Multiplicative sizing refinement over `region`, clamped at `floor`.
 *
 * @param sizing `double&(size_t vid)` -- the vertex's sizing scalar, by reference
 * @return the vertices whose sizing was actually lowered (the seeds for the gradation pass)
 */
template <class Sizing>
std::vector<size_t> apply_sizing_refinement(
    const std::unordered_set<size_t>& region,
    double factor,
    double floor,
    Sizing sizing)
{
    std::vector<size_t> refined;
    refined.reserve(region.size());
    for (const size_t v : region) {
        double& s = sizing(v);
        const double ns = std::max(floor, s * factor);
        if (ns < s) {
            s = ns;
            refined.push_back(v);
        }
    }
    return refined;
}

/**
 * @brief Monotone (only-decreasing) gradation smoothing of a sizing field.
 *
 * Enforces sizing[v] <= grade * sizing[u] for every edge (u,v), propagating outward from
 * `seeds` with a min-relaxation (Dijkstra/Bellman-Ford style): vertex u caps each neighbor
 * at grade * sizing[u]. It only ever decreases sizings, so it spreads more refinement
 * outward without raising the already-refined seed values, avoiding sharp resolution jumps
 * that make operations ill-conditioned. Sizings are always in (0, 1], so once
 * grade*sizing[u] >= 1 the cap can no longer lower any neighbor and propagation stops --
 * this is what bounds the halo.
 */
template <class Sizing, class OneRing>
void gradation_smooth_sizing(
    double grade,
    const std::vector<size_t>& seeds,
    Sizing sizing,
    OneRing one_ring)
{
    if (grade <= 1.0) {
        return;
    }

    std::queue<size_t> q;
    for (const size_t v : seeds) {
        q.push(v);
    }
    while (!q.empty()) {
        const size_t u = q.front();
        q.pop();
        const double cap = grade * sizing(u);
        if (cap >= 1.0) {
            continue;
        }
        for (const size_t w : one_ring(u)) {
            double& sw = sizing(w);
            if (sw > cap) {
                sw = cap;
                q.push(w);
            }
        }
    }
}

/**
 * @brief The vertices incident to at least one cell whose quality reaches `thr`.
 *
 * Used by the skip-good-regions filter to restrict smoothing to non-good regions:
 * smoothing a vertex surrounded by good cells does nothing, so skipping it is free.
 *
 * @param cell_vids `Range(size_t cid)` -- the cell's vertex ids
 */
template <class IsValid, class Quality, class CellVids>
std::vector<size_t> active_vertices(
    size_t n_verts,
    size_t n_cells,
    IsValid is_valid,
    Quality quality,
    CellVids cell_vids,
    double thr)
{
    std::vector<char> seen(n_verts, 0);
    std::vector<size_t> out;
    for (size_t cid = 0; cid < n_cells; ++cid) {
        if (!is_valid(cid)) {
            continue;
        }
        if (quality(cid) < thr) {
            continue;
        }
        for (const size_t v : cell_vids(cid)) {
            if (!seen[v]) {
                seen[v] = 1;
                out.push_back(v);
            }
        }
    }
    return out;
}

} // namespace wmtk::utils
