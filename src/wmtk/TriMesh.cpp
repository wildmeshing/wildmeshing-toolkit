#include <Mesh.hpp>

namespace wmtk {
long TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv[tuple.m_global_cid * 3 + tuple.m_local_vid]; break;
    case PrimitiveType::Edge: return m_fe[tuple.m_global_cid * 3 + tuple.m_local_eid]; break;
    case PrimitiveType::Triangle: return tuple.m_global_cid; break;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override
{
    bool ccw = is_ccw(tuple);
    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            (tuple.m_local_vid + ccw ? 1 : 2) % 3,
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);

    case PrimitiveType::Edge:
        return Tuple(
            tuple.m_local_vid,
            (tuple.m_local_eid + ccw ? 2 : 1) % 3,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Triangle: {
        long gvid = id(tuple, 0);
        long geid = id(tuple, 1);
        long gcid_new = m_ff[tuple.m_global_cid * 3 + tuple.m_local_eid];
        long lvid_new, leid_new;
        for (long i = 0; i < 3; ++i) {
            if (m_fe[gcid_new * 3 + i] == geid) {
                leid_new = m_fe[gcid_new * 3 + i];
            }
            if (m_fv[gcid_new * 3 + i] == gvid) {
                lvid_new = m_fv[gcid_new * 3 + i];
            }
        }
        return Tuple(lvid_new, leid_new, tuple.m_local_fid, gcid_new, tuple.m_hash);
    }
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}
bool TriMesh::is_ccw(const Tuple& tuple) const override
{
    if (m_fv[tuple.m_global_cid * 3 + (tuple.m_local_eid + 1) % 3] == id(tuple, 0))
        return true;
    else
        return false;
}
} // namespace wmtk