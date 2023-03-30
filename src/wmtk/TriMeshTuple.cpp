
#include "TriMesh.h"
using namespace wmtk;
void TriMeshTuple::update_hash(const TriMesh& m)
{
    assert(m_fid < m.m_tri_connectivity.size());
    m_hash = m.m_tri_connectivity[m_fid].hash;
}

std::string TriMeshTuple::info() const
{
    return fmt::format("tuple: v{} e{} f{} (h{})", m_vid, m_eid, m_fid, m_hash);
}

void TriMeshTuple::print_info() const
{
    logger().trace("{}", info());
}

size_t TriMeshTuple::eid_unsafe(const TriMesh& m) const
{
    return m_fid * 3 + m_eid;
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
    return m_fid * 3 + m_eid;
}


TriMeshTuple TriMeshTuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];

    TriMeshTuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == v0 || m_vid == v2);
        loc.m_vid = m_vid == v0 ? v2 : v0;
        break;
    case 2:
        assert(m_vid == v0 || m_vid == v1);
        loc.m_vid = m_vid == v0 ? v1 : v0;
        break;
    default:;
    }
    assert(loc.is_valid(m));

    return loc;
}

TriMeshTuple TriMeshTuple::switch_edge(const TriMesh& m) const
{
    assert(is_valid(m));

    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    TriMeshTuple loc = *this;
    switch (lvid) {
    case 0:
        assert(m_eid == 1 || m_eid == 2);
        loc.m_eid = m_eid == 1 ? 2 : 1;
        break;
    case 1:
        assert(m_eid == 0 || m_eid == 2);
        loc.m_eid = m_eid == 0 ? 2 : 0;
        break;
    case 2:
        assert(m_eid == 0 || m_eid == 1);
        loc.m_eid = m_eid == 0 ? 1 : 0;
        break;
    default:;
    }
    assert(loc.is_valid(m));
    return loc;
}

std::optional<TriMeshTuple> TriMeshTuple::switch_face(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    std::vector<size_t> v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
    std::vector<size_t> v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

    std::sort(v0_fids.begin(), v0_fids.end());
    std::sort(v1_fids.begin(), v1_fids.end());
    std::vector<int> fids;
    std::set_intersection(
        v0_fids.begin(),
        v0_fids.end(),
        v1_fids.begin(),
        v1_fids.end(),
        std::back_inserter(fids)); // make sure this is correct
    assert(fids.size() == 1 || fids.size() == 2);

    if (fids.size() == 1) return {};

    TriMeshTuple loc = *this;

    // There is a triangle on the other side
    if (fids.size() == 2) {
        // Find the fid of the triangle on the other side
        size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];
        loc.m_fid = fid2;

        // Get sorted local indices of the two vertices in the new triangle
        size_t lv0_2 = m.m_tri_connectivity[fid2].find(v0);
        assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
        size_t lv1_2 = m.m_tri_connectivity[fid2].find(v1);
        assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

        if (lv0_2 > lv1_2) std::swap(lv0_2, lv1_2);

        // Assign the edge id depending on the table
        if (lv0_2 == 0 && lv1_2 == 1) {
            loc.m_eid = 2;
        } else if (lv0_2 == 1 && lv1_2 == 2) {
            loc.m_eid = 0;
        } else if (lv0_2 == 0 && lv1_2 == 2) {
            loc.m_eid = 1;
        } else {
            assert(false);
        }

        loc.update_hash(m);
    }
    assert(loc.is_valid(m));
    return loc;
}

bool TriMeshTuple::is_ccw(const TriMesh& m) const
{
    if (m.m_tri_connectivity[m_fid][(m_eid + 1) % 3] == m_vid)
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

    if (m.m_vertex_connectivity[m_vid].m_is_removed) {
        return false;
    }

    if (m.m_tri_connectivity[m_fid].m_is_removed) {
        return false;
    }

    // Condition 3: tuple m_hash check
    if (m_hash != m.m_tri_connectivity[m_fid].hash) {
        // assert(false);
        return false;
    }
#ifndef NDEBUG
    //  Condition 0: Elements exist
    assert(m_vid < m.vert_capacity());
    assert(m_eid <= 2);
    assert(m_fid <= m.tri_capacity());

    // Condition 1: tid and vid are consistent
    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    // Condition 2: eid is valid
    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == v0 || m_vid == v2); break;
    case 2: assert(m_vid == v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}
