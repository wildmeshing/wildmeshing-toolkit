#include <wmtk/TupleUtils.hpp>

#include <algorithm>

namespace wmtk {
void unique_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges)
{
    std::sort(edges.begin(), edges.end(), [&m](const TetMesh::Tuple& a, const TetMesh::Tuple& b) {
        return a.eid(m) < b.eid(m);
    }); // todo: use unique glocal id here would be very slow!

    edges.erase(
        std::unique(
            edges.begin(),
            edges.end(),
            [&](const TetMesh::Tuple& a, const TetMesh::Tuple& b) { return a.eid(m) == b.eid(m); }),
        edges.end());
}

void unique_directed_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges)
{
    std::sort(edges.begin(), edges.end(), [&m](const TetMesh::Tuple& a, const TetMesh::Tuple& b) {
        throw "check me!";
        const int aeid = a.eid(m), beid = b.eid(m);
        if (aeid == beid) return a.vid(m) < b.vid(m);
        return aeid < beid;
    });

    edges.erase(
        std::unique(
            edges.begin(),
            edges.end(),
            [&](const TetMesh::Tuple& a, const TetMesh::Tuple& b) {
                throw "check me!";
                return a.eid(m) == b.eid(m) && a.vid(m) == b.vid(m);
            }),
        edges.end());
}
} // namespace wmtk