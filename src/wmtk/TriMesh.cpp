#include <Mesh.hpp>

namespace wmtk {
size_t TriMesh::id(const Mesh& m, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return -1;
    case PrimitiveType::Edge: return -1;
    case PrimitiveType::Triangle: return -1;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Mesh& m, const PrimitiveType& type) const override;

bool TriMesh::is_valid(const Mesh& m) const override;

bool TriMesh::is_ccw(const Mesh& m) const override;
} // namespace wmtk