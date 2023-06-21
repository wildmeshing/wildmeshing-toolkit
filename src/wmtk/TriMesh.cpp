#include <Mesh.hpp>

namespace wmtk {
size_t TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
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
    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            tuple.m_local_vid,
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
        break;
    case PrimitiveType::Edge:
        return Tuple(
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_local_vid,
            tuple.m_global_cid,
            tuple.m_hash);
        break;
    case PrimitiveType::Triangle:
        return Tuple(
            tuple.m_local_fid,
            tuple.m_local_vid,
            tuple.m_local_eid,
            tuple.m_global_cid,
            tuple.m_hash);
        break;
    default: throw std::runtime_error("Tuple switch: Invalid primitive type");
    }

    bool TriMesh::is_ccw(const Tuple& tuple) const override
    {
        if (m_fv[tuple.m_globacl_cid + (tuple.m_local_eid + 1) % 3] == id(tuple, 0))
            return true;
        else
            return false;
    }
}
} // namespace wmtk