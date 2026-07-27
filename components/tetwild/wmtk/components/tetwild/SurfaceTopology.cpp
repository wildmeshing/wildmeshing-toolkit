#include "TetWildMesh.h"

#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace wmtk::components::tetwild {

namespace {
// A tiny union-find over dense [0, n) indices, used to count connected
// components of the surface and of its boundary-edge subgraph.
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

} // namespace

TetWildMesh::SurfaceTopoSignature TetWildMesh::surface_topology_signature() const
{
    // Collect the tracked surface triangles (canonical faces, deduplicated by
    // global fid), each as a sorted vertex triple.
    std::vector<simplex::Face> faces;
    for (size_t i = 0; i < tet_capacity(); ++i) {
        const Tuple tt = tuple_from_tet(i);
        if (!tt.is_valid(*this)) {
            continue;
        }
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(i, j);
            const size_t fid = f.fid(*this);
            if (fid != 4 * i + j) {
                continue; // visit each face once (canonical)
            }
            if (!m_face_attribute[fid].m_is_surface_fs) {
                continue;
            }
            faces.emplace_back(simplex_from_face(f));
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
        DSU dsu;
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

    // Boundary loops: a boundary edge is incident to exactly one surface face.
    // On a manifold-with-boundary surface each connected component of the
    // boundary-edge subgraph is a simple cycle, so #components == #loops.
    {
        DSU bd;
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

void TetWildMesh::warn_if_surface_topology_changed(
    const SurfaceTopoSignature& before,
    const char* where) const
{
    const SurfaceTopoSignature after = surface_topology_signature();
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

} // namespace wmtk::components::tetwild
