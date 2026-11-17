#include "ManifoldCheck.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <numeric>
#include <optional>

namespace wmtk::utils {

namespace {

/// Union-find over the cells around a simplex, by their index there.
struct Groups
{
    std::vector<size_t> parent;

    explicit Groups(const size_t n)
        : parent(n)
    {
        std::iota(parent.begin(), parent.end(), size_t(0));
    }
    size_t find(size_t x)
    {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void join(const size_t a, const size_t b) { parent[find(a)] = find(b); }
    /// whether the members all lie in one group (true if there are none)
    bool one_group(const std::vector<size_t>& members)
    {
        return std::all_of(members.begin(), members.end(), [&](const size_t x) {
            return find(x) == find(members.front());
        });
    }
};

/// Joins the cells of K that hold a facet in common, and the cells outside K that do. Each facet
/// is given by the cells holding it (indices); a facet held by one cell lies on the mesh's
/// boundary, and the cell outside K holding it is joined to `outside`, if given.
template <typename Facets>
void join_through_facets(
    Groups& groups,
    const Facets& facets,
    const std::vector<char>& in,
    const std::optional<size_t> outside)
{
    for (const auto& [key, holders] : facets) {
        if (holders.size() == 1) {
            if (outside && !in[holders[0]]) {
                groups.join(holders[0], *outside);
            }
            continue;
        }
        for (size_t k = 1; k < holders.size(); ++k) {
            if (in[holders[0]] == in[holders[k]]) {
                groups.join(holders[0], holders[k]);
            }
        }
    }
}

/// The indices of the cells in K (or outside it).
std::vector<size_t> members(const std::vector<char>& in, const bool of_K)
{
    std::vector<size_t> ids;
    for (size_t i = 0; i < in.size(); ++i) {
        if (bool(in[i]) == of_K) {
            ids.push_back(i);
        }
    }
    return ids;
}

bool tri_vertex_is_manifold(const TriMesh& m, const size_t vid, const CellTest& in_K)
{
    const std::vector<size_t>& ring = m.get_one_ring_fids_for_vertex(vid);
    std::vector<char> in(ring.size());
    // the edges at v, by their other vertex: the faces holding each
    std::map<size_t, std::vector<size_t>> edges;
    for (size_t i = 0; i < ring.size(); ++i) {
        in[i] = in_K(ring[i]);
        for (const size_t u : m.oriented_tri_vids(ring[i])) {
            if (u != vid) {
                edges[u].push_back(i);
            }
        }
    }
    Groups groups(ring.size());
    join_through_facets(groups, edges, in, std::nullopt);
    return groups.one_group(members(in, true));
}

bool tet_edge_is_manifold(const TetMesh& m, const size_t a, const size_t b, const CellTest& in_K)
{
    std::vector<size_t> around;
    for (const TetMesh::Tuple& t : m.get_incident_tets_for_edge(a, b)) {
        around.push_back(t.tid(m));
    }
    std::vector<char> in(around.size());
    // the faces holding the edge, by their third vertex: the tets holding each
    std::map<size_t, std::vector<size_t>> faces;
    for (size_t i = 0; i < around.size(); ++i) {
        in[i] = in_K(around[i]);
        for (const size_t u : m.oriented_tet_vids(around[i])) {
            if (u != a && u != b) {
                faces[u].push_back(i);
            }
        }
    }
    Groups groups(around.size());
    join_through_facets(groups, faces, in, std::nullopt);
    return groups.one_group(members(in, true));
}

bool tet_vertex_is_manifold(const TetMesh& m, const size_t vid, const CellTest& in_K)
{
    const std::vector<size_t>& ring = m.get_one_ring_tids_for_vertex(vid);
    std::vector<char> in(ring.size());
    // the faces at v, by their two other vertices: the tets holding each
    std::map<std::array<size_t, 2>, std::vector<size_t>> faces;
    for (size_t i = 0; i < ring.size(); ++i) {
        in[i] = in_K(ring[i]);
        std::vector<size_t> others;
        for (const size_t u : m.oriented_tet_vids(ring[i])) {
            if (u != vid) {
                others.push_back(u);
            }
        }
        std::sort(others.begin(), others.end());
        faces[{{others[0], others[1]}}].push_back(i);
        faces[{{others[0], others[2]}}].push_back(i);
        faces[{{others[1], others[2]}}].push_back(i);
    }
    // index ring.size() stands for the space outside the mesh
    const size_t outside = ring.size();
    Groups groups(ring.size() + 1);
    join_through_facets(groups, faces, in, outside);
    std::vector<size_t> out = members(in, false);
    const bool on_boundary =
        std::any_of(faces.begin(), faces.end(), [](const auto& f) { return f.second.size() == 1; });
    if (on_boundary) {
        out.push_back(outside);
    }
    return groups.one_group(members(in, true)) && groups.one_group(out);
}

} // namespace


bool vertex_is_manifold(const TriMesh& m, const TriMesh::Tuple& v, const CellTest& in_K)
{
    return tri_vertex_is_manifold(m, v.vid(m), in_K);
}

bool edge_is_manifold(const TetMesh& m, const TetMesh::Tuple& e, const CellTest& in_K)
{
    return tet_edge_is_manifold(m, e.vid(m), e.switch_vertex(m).vid(m), in_K);
}

bool vertex_is_manifold(const TetMesh& m, const TetMesh::Tuple& v, const CellTest& in_K)
{
    return tet_vertex_is_manifold(m, v.vid(m), in_K);
}

std::vector<size_t>
nonmanifold_vertices(const TriMesh& m, const std::vector<size_t>& cells, const CellTest& in_K)
{
    std::vector<size_t> vertices;
    for (const size_t fid : cells) {
        for (const size_t v : m.oriented_tri_vids(fid)) {
            vertices.push_back(v);
        }
    }
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());

    std::vector<size_t> found;
    for (const size_t v : vertices) {
        if (!tri_vertex_is_manifold(m, v, in_K)) {
            found.push_back(v);
        }
    }
    return found;
}

std::vector<size_t>
nonmanifold_vertices(const TetMesh& m, const std::vector<size_t>& cells, const CellTest& in_K)
{
    std::vector<size_t> vertices;
    std::vector<std::array<size_t, 2>> edges;
    for (const size_t tid : cells) {
        const std::array<size_t, 4> vs = m.oriented_tet_vids(tid);
        for (int i = 0; i < 4; ++i) {
            vertices.push_back(vs[i]);
            for (int j = i + 1; j < 4; ++j) {
                edges.push_back({{std::min(vs[i], vs[j]), std::max(vs[i], vs[j])}});
            }
        }
    }
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

    std::vector<size_t> found;
    for (const size_t v : vertices) {
        if (!tet_vertex_is_manifold(m, v, in_K)) {
            found.push_back(v);
        }
    }
    for (const auto& [a, b] : edges) {
        if (!tet_edge_is_manifold(m, a, b, in_K)) {
            found.push_back(a);
            found.push_back(b);
        }
    }
    std::sort(found.begin(), found.end());
    found.erase(std::unique(found.begin(), found.end()), found.end());
    return found;
}

} // namespace wmtk::utils
