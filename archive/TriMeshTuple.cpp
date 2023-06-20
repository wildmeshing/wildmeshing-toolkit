#include "TriMesh.h"
// Disable compiler warnings before including third party code
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcomment"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcomment"
#endif

namespace wmtk {
void TriMeshTuple::update_hash(const TriMesh& m)
{
    assert(m_fid < m.m_tri_connectivity.size());
    m_hash = m.m_tri_connectivity[m_fid].hash;
}

std::string TriMeshTuple::info() const
{
    return fmt::format("tuple: v{} e{} f{} (h{})", m_vid, m_local_eid, m_fid, m_hash);
}

void TriMeshTuple::print_info() const
{
    logger().trace("{}", info());
}

size_t TriMeshTuple::eid_unsafe(const TriMesh& m) const
{
    return m_fid * 3 + m_local_eid;
}

size_t TriMeshTuple::eid(const TriMesh& m) const
{
    if (switch_face(m).has_value()) {
        size_t fid2 = switch_face(m)->fid(m);
        size_t min_fid = std::min(m_fid, fid2);
        if (min_fid == fid2) {
            assert(fid2 < m.m_tri_connectivity.size());
            int i = m.m_tri_connectivity[fid2].find(m_vid);
            int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
            return min_fid * 3 + 3 - i - j;
        }
    }
    return m_fid * 3 + m_local_eid;
}


TriMeshTuple TriMeshTuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));
    const auto& tri_con = m.m_tri_connectivity[m_fid].m_indices;
    const bool ccw = is_ccw(m);


    // 1|\
    // 2| \0
    //  |  \
    // 0|___\2
    //   1
    //

    TriMeshTuple loc = *this;
    loc.m_vid = tri_con[(m_local_eid + (ccw ? 2 : 1)) % 3];

    assert(loc.is_valid(m));

    return loc;
}

TriMeshTuple TriMeshTuple::switch_edge(const TriMesh& m) const
{
    assert(is_valid(m));

    //  lvid = 0 -> eid == 1 <=> 2
    //  lvid = 1 -> eid == 2 <=> 0
    //  lvid = 2 -> eid == 0 <=> 1

    // 1|\
    // 2| \0
    //  |  \
    // 0|___\2
    //   1
    //
    TriMeshTuple loc = *this;
    const bool ccw = is_ccw(m);
    loc.m_local_eid = (m_local_eid + (ccw ? 2 : 1)) % 3;
    assert(loc.is_valid(m));
    return loc;
}

std::optional<TriMeshTuple> TriMeshTuple::switch_face(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    const std::vector<size_t>& v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
    const std::vector<size_t>& v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

    assert(std::is_sorted(v0_fids.begin(), v0_fids.end()));
    assert(std::is_sorted(v1_fids.begin(), v1_fids.end()));
    std::array<size_t, 2> fids;
    auto output_end = std::set_intersection(

        v0_fids.begin(),
        v0_fids.end(),
        v1_fids.begin(),
        v1_fids.end(),
        fids.begin()); // make sure this is correct
    const size_t isect_size = std::distance(fids.begin(), output_end);
    assert(isect_size == 1 || isect_size == 2);

    if (isect_size != 2) return {};


    // There is a triangle on the other side
    // Find the fid of the triangle on the other side
    const size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];

    // Get sorted local indices of the two vertices in the new triangle
    const auto& fid2_tri_con = m.m_tri_connectivity[fid2];

    size_t local_eid = 4;
    for (size_t j = 0; j < 3; ++j) {
        if (fid2_tri_con[j] != v0 && fid2_tri_con[j] != v1) {
            local_eid = j;
            break;
        }
    }
    assert(local_eid != 4);
    const TriMeshTuple loc(this->vid(m), local_eid, fid2, m);
#if !defined(NDEBUG)
    size_t lv0_2 = fid2_tri_con.find(v0);
    size_t lv1_2 = fid2_tri_con.find(v1);
    // make sure the edges are legit values
    assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
    assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

    // make sure the local eid is the "other" edge
    assert(loc.m_local_eid + lv0_2 + lv1_2 == 3);
    assert(loc.is_valid(m));
#endif
    return loc;
}

bool TriMeshTuple::is_ccw(const TriMesh& m) const
{
    if (m.m_tri_connectivity[m_fid][(m_local_eid + 1) % 3] == m_vid)
        return true;
    else
        return false;
}
bool TriMeshTuple::is_valid(const TriMesh& m) const
{
    if (m_fid >= m.m_tri_connectivity.size()) {
        return false;
    }
    if (m_vid >= m.m_vertex_connectivity.size()) {
        return false;
    }

    if (m_fid + 1 == 0) {
        return false;
    }

    const auto& vert_con = m.m_vertex_connectivity[m_vid];
    if (vert_con.m_is_removed) {
        return false;
    }

    const auto& tri_con = m.m_tri_connectivity[m_fid];
    if (tri_con.m_is_removed) {
        return false;
    }

    // Condition 3: tuple m_hash check
    if (m_hash != tri_con.hash) {
        // assert(false);
        return false;
    }
#ifndef NDEBUG
    //  Condition 0: Elements exist
    assert(m_vid < m.vert_capacity());
    assert(m_local_eid <= 2);
    assert(m_fid <= m.tri_capacity());

    // Condition 1: tid and vid are consistent
    const int lvid = tri_con.find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    // Condition 2: eid is valid
    const int v0 = tri_con[0];
    const int v1 = tri_con[1];
    const int v2 = tri_con[2];
    switch (m_local_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == v0 || m_vid == v2); break;
    case 2: assert(m_vid == v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}
} // namespace wmtk
