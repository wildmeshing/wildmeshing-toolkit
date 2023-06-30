#include "PointMesh.hpp"
namespace wmtk {
Tuple PointMesh::vertex_tuple_from_id(long id) const
{
    return Tuple(-1, -1, -1, id, get_cell_hash_slow(id));
}

PointMesh::PointMesh()
    : Mesh(1)
{}
Tuple PointMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{
    throw std::runtime_error("Tuple switch: Invalid primitive type");
    return tuple;
}
bool PointMesh::is_ccw(const Tuple&) const
{
    // trivial orientation so nothing can happen
    return true;
}
bool PointMesh::is_boundary(const Tuple&) const
{
    // every point is on the interior as it has no boundary simplices
    return false;
}

void initialize(long count);


bool PointMesh::is_valid(const Tuple& tuple) const
{
    return true;
}

long PointMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    switch (type) {
    case PrimitiveType::Vertex:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

Tuple PointMesh::tuple_from_id(const PrimitiveType type, const long gid) const
{
    if (type != PrimitiveType::Vertex) {
        throw std::runtime_error("Tuple switch: Invalid primitive type");
    }
    return vertex_tuple_from_id(gid);
}
} // namespace wmtk
