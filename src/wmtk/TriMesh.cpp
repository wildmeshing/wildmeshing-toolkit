#include <Mesh.hpp>

namespace wmtk {
size_t TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv_handle[tuple.m_global_cid * 3 + tuple.m_locak_vid];
    case PrimitiveType::Edge: return -1;
    case PrimitiveType::Triangle: return -1;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;

bool TriMesh::is_valid(const Tuple& tuple) const override;

bool TriMesh::is_ccw(const Tuple& tuple) const override;
} // namespace wmtk