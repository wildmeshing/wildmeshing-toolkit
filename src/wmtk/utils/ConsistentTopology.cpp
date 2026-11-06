#include "ConsistentTopology.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace wmtk::utils {

namespace {

/// Whether any of `cells` but `self` is in the region.
bool any_in_region(
    const std::vector<size_t>& cells,
    const size_t self,
    const std::function<bool(size_t)>& in_region)
{
    return std::any_of(cells.begin(), cells.end(), [&](const size_t c) {
        return c != self && in_region(c);
    });
}

} // namespace

bool tri_consistent_topology(
    const TriMesh& m,
    const size_t fid,
    const std::function<bool(size_t)>& in_region)
{
    int n_vs_in = 0;
    for (const size_t v : m.oriented_tri_vids(fid)) {
        n_vs_in += any_in_region(m.get_one_ring_fids_for_vertex(v), fid, in_region);
    }
    int n_edges_in = 0;
    for (size_t j = 0; j < 3; ++j) {
        const auto opp = m.tuple_from_edge(fid, j).switch_face(m);
        n_edges_in += opp.has_value() && in_region(opp->fid(m));
    }
    return (n_vs_in == 2 && n_edges_in == 1) || (n_vs_in == 3 && n_edges_in == 2);
}

bool tet_consistent_topology(
    const TetMesh& m,
    const size_t tid,
    const std::function<bool(size_t)>& in_region)
{
    const std::array<size_t, 4> vs = m.oriented_tet_vids(tid);
    int n_vs_in = 0;
    for (const size_t v : vs) {
        n_vs_in += any_in_region(m.get_one_ring_tids_for_vertex(v), tid, in_region);
    }
    std::vector<size_t> opposite; // the corner opposite each face in the region
    for (int j = 0; j < 4; ++j) {
        const TetMesh::Tuple f = m.tuple_from_face(tid, j);
        const auto opp = f.switch_tetrahedron(m);
        if (!opp.has_value() || !in_region(opp->tid(m))) {
            continue;
        }
        const std::array<size_t, 3> fv = m.get_face_vids(f);
        for (const size_t v : vs) {
            if (std::find(fv.begin(), fv.end(), v) == fv.end()) {
                opposite.push_back(v);
            }
        }
    }
    if (n_vs_in == 3) {
        return opposite.size() == 1;
    }
    if (n_vs_in == 4 && opposite.size() == 3) {
        return true;
    }
    if (n_vs_in == 4 && opposite.size() == 2) {
        // the two opposite corners span the edge on neither face
        return !any_in_region(
            m.get_incident_tids_for_edge(opposite[0], opposite[1]),
            tid,
            in_region);
    }
    return false;
}

} // namespace wmtk::utils
