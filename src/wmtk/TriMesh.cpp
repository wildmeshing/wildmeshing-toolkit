#include <Mesh.hpp>

namespace wmtk {
size_t TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv[tuple.m_global_cid * 3 + tuple.m_local_vid];
    case PrimitiveType::Edge: return m_fe[tuple.m_global_cid * 3 + tuple.m_local_eid];
    case PrimitiveType::Triangle: return tuple.m_global_cid;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override {}

bool TriMesh::is_ccw(const Tuple& tuple) const override;
} // namespace wmtk