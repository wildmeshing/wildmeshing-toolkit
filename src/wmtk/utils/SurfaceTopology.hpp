#pragma once

#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <cstddef>
#include <map>
#include <set>
#include <vector>

namespace wmtk::utils {

/**
 * @brief A topological fingerprint of a tracked surface inside a tet mesh.
 *
 * Cheap-to-compare summary used to assert that surface-modifying operations (surface edge
 * flips) do not change the surface topology: number of connected components, surface V/E/F,
 * Euler characteristic, and number of boundary loops. A valid surface diagonal flip leaves
 * all of these invariant. O(#surface faces); only used by tests and by the
 * check_surface_topology debug option.
 */
struct SurfaceTopoSignature
{
    long long components = 0;
    long long V = 0;
    long long E = 0;
    long long F = 0;
    long long euler = 0; // V - E + F
    long long boundary_loops = 0;
    bool operator==(const SurfaceTopoSignature&) const = default;
};

namespace detail {
// A tiny union-find over dense [0, n) indices, used to count connected components of the
// surface and of its boundary-edge subgraph.
struct DSU
{
    std::vector<long long> parent;
    void init(long long n)
    {
        parent.resize(n);
        for (long long i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }
    long long find(long long x)
    {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(long long a, long long b) { parent[find(a)] = find(b); }
};
} // namespace detail

/**
 * @brief Compute the signature of the surface tracked by `is_surface`.
 *
 * @param m          a TetMesh-like mesh (cell_capacity / tuple_from_cell / tuple_from_face /
 *                   simplex_from_face / FACES_PER_CELL)
 * @param is_surface `bool(size_t fid)` -- whether the global face fid is on the surface
 */
template <class Mesh, class IsSurfaceFace>
SurfaceTopoSignature surface_topology_signature(const Mesh& m, IsSurfaceFace is_surface)
{
    using Tuple = typename Mesh::Tuple;
    constexpr size_t n_faces = Mesh::FACES_PER_CELL;

    // Collect the tracked surface triangles (canonical faces, deduplicated by global fid),
    // each as a sorted vertex triple.
    std::vector<simplex::Face> faces;
    for (size_t i = 0; i < m.cell_capacity(); ++i) {
        if (!m.tuple_from_cell(i).is_valid(m)) {
            continue;
        }
        for (size_t j = 0; j < n_faces; ++j) {
            const Tuple f = m.tuple_from_face(i, j);
            const size_t fid = f.fid(m);
            if (fid != n_faces * i + j) {
                continue; // visit each face once (canonical)
            }
            if (!is_surface(fid)) {
                continue;
            }
            faces.emplace_back(m.simplex_from_face(f));
        }
    }

    SurfaceTopoSignature sig;
    sig.F = static_cast<long long>(faces.size());

    // Dense-index the surface vertices; count edge incidences.
    std::map<size_t, long long> vidx;
    auto index_of = [&](size_t v) -> long long {
        auto it = vidx.find(v);
        if (it != vidx.end()) {
            return it->second;
        }
        const long long id = static_cast<long long>(vidx.size());
        vidx.emplace(v, id);
        return id;
    };
    std::map<simplex::Edge, int> edge_count;
    for (const simplex::Face& f : faces) {
        const auto& vs = f.vertices();
        index_of(vs[0]);
        index_of(vs[1]);
        index_of(vs[2]);
        edge_count[simplex::Edge(vs[0], vs[1])]++;
        edge_count[simplex::Edge(vs[1], vs[2])]++;
        edge_count[simplex::Edge(vs[0], vs[2])]++;
    }
    sig.V = static_cast<long long>(vidx.size());
    sig.E = static_cast<long long>(edge_count.size());
    sig.euler = sig.V - sig.E + sig.F;

    // Connected components of the surface (vertices joined by any surface edge).
    {
        detail::DSU dsu;
        dsu.init(sig.V);
        for (const simplex::Face& f : faces) {
            const auto& vs = f.vertices();
            const long long a = vidx[vs[0]], b = vidx[vs[1]], c = vidx[vs[2]];
            dsu.unite(a, b);
            dsu.unite(b, c);
        }
        std::set<long long> roots;
        for (long long i = 0; i < sig.V; ++i) {
            roots.insert(dsu.find(i));
        }
        sig.components = static_cast<long long>(roots.size());
    }

    // Boundary loops: a boundary edge is incident to exactly one surface face. On a
    // manifold-with-boundary surface each connected component of the boundary-edge subgraph
    // is a simple cycle, so #components == #loops.
    {
        detail::DSU bd;
        bd.init(sig.V);
        std::set<long long> bverts;
        for (const auto& [e, cnt] : edge_count) {
            if (cnt != 1) {
                continue;
            }
            const long long ia = vidx[e.vertices()[0]], ib = vidx[e.vertices()[1]];
            bd.unite(ia, ib);
            bverts.insert(ia);
            bverts.insert(ib);
        }
        std::set<long long> broots;
        for (long long v : bverts) {
            broots.insert(bd.find(v));
        }
        sig.boundary_loops = static_cast<long long>(broots.size());
    }

    return sig;
}

/**
 * @brief Compare two surface signatures and log an error if they differ. Used to guard
 * swap passes that can flip surface edges.
 */
inline void warn_if_surface_topology_changed(
    const SurfaceTopoSignature& before,
    const SurfaceTopoSignature& after,
    const char* where)
{
    if (after == before) {
        return;
    }
    logger().error(
        "[surface-swap] surface topology CHANGED in {}: components {}->{}, V {}->{}, "
        "E {}->{}, F {}->{}, euler {}->{}, boundary_loops {}->{}",
        where,
        before.components,
        after.components,
        before.V,
        after.V,
        before.E,
        after.E,
        before.F,
        after.F,
        before.euler,
        after.euler,
        before.boundary_loops,
        after.boundary_loops);
}

} // namespace wmtk::utils
