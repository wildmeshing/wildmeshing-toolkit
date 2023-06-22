#include <Mesh.hpp>

namespace wmtk {
long TetMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return m_tv[tuple.m_global_cid * 4 + tuple.m_local_vid]; break;
    case PrimitiveType::Edge: return m_te[tuple.m_global_cid * 6 + tuple.m_local_eid]; break;
    case PrimitiveType::Triangle: return m_tf[tuple.m_global_cid * 3 + tuple.m_local_fid]; break;
    case PrimitiveType::Tetrahedron: return tuple.m_global_cid; break;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TetMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override
{
    assert(is_valid(tuple));

    // Table of vertices corresponding to edges of the tetrahedron
    static const edge_table[6][2] =
    {
        {0,1},
        {0,2},
        {0,3},
        {1,2},
        {1,3},
        {2,3}
    }

    // Table of oriented vertices corresponding to faces of the tetrahedron
    // Face i does not contain vertex i
    static const face_table[4][3] =
    {
        {1,2,3},
        {0,3,2},
        {0,1,3},
        {0,2,1}
    }

    // bool ccw = is_ccw(tuple);
    switch (type) {
    case PrimitiveType::Vertex: // DONE
        bool first = tuple.m_local_vid == edge_table[tuple.m_local_eid][0];
        assert(tuple.m_local_vid == first?edge_table[tuple.m_local_eid][0]:edge_table[tuple.m_local_eid][1]);
        return Tuple(
            first ? edge_table[tuple.m_local_eid][1]:edge_table[tuple.m_local_eid][2],
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
    case PrimitiveType::Face:
        return Tuple(
            tuple.m_local_vid,
            (tuple.m_local_eid + ccw ? 2 : 1) % 3,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Tetrahedron: {
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
bool TetMesh::is_ccw(const Tuple& tuple) const override
{
    if (m_fv[tuple.m_global_cid * 3 + (tuple.m_local_eid + 1) % 3] == id(tuple, 0))
        return true;
    else
        return false;
}
} // namespace wmtk