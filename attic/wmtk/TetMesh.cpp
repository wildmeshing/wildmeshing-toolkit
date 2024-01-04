#include <Mesh.hpp>
#include

namespace wmtk {
int64_t TetMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
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
    int offset = (tuple.m_local_vid * 6 * 4 + tuple.m_local_eid * 6 + tuple.m_local_fid) * 3;
    // bool ccw = is_ccw(tuple);
    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            3d_tuple_table_vertex [offset + 0],
            3d_tuple_table_vertex [offset + 1],
            3d_tuple_table_vertex [offset + 2],
            tuple.m_global_cid,
            tuple.m_hash);

    case PrimitiveType::Edge:
        return Tuple(
            3d_tuple_table_edge [offset + 0],
            3d_tuple_table_edge [offset + 1],
            3d_tuple_table_edge [offset + 2],
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Face:
        return Tuple(
            3d_tuple_table_face [offset + 0],
            3d_tuple_table_face [offset + 1],
            3d_tuple_table_face [offset + 2],
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Tetrahedron: {
        // TODO
    }
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}
bool TetMesh::is_ccw(const Tuple& tuple) const override
{
    // TODO
}
} // namespace wmtk