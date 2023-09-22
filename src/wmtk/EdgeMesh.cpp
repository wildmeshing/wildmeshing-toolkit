#include "EdgeMesh"
namespace wmtk {
Tuple EdgeMesh::vertex_tuple_from_id(long id) const
{
    return Tuple(-1, -1, -1, id, get_cell_hash_slow(id));
}

EdgeMesh::EdgeMesh()
    : Mesh(0)
{}


EdgeMesh::EdgeMesh(long size)
    : PointMesh()
{
    initialize(size);
}

Tuple EdgeMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    throw std::runtime_error("Tuple switch: Invalid primitive type");
    return tuple;
}
bool EdgeMesh::is_ccw(const Tuple&) const
{
    // trivial orientation so nothing can happen
    return true;
}
bool EdgeMesh::is_boundary(const Tuple&) const
{
    // every point is on the interior as it has no boundary simplices
    return false;
}

bool EdgeMesh::is_boundary_vertex(const Tuple&) const
{
    // every point is on the interior as it has no boundary simplices
    return false;
}

void EdgeMesh::initialize(long count)
{
    set_capacities({count});
    reserve_attributes_to_fit();
    Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        v_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}


bool EdgeMesh::is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const
{
    if (tuple.is_null()) return false;
    return true;
    return Mesh::is_hash_valid(tuple, hash_accessor);

}

long EdgeMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return tuple.m_global_cid;
    case PrimitiveType::Edge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

Tuple EdgeMesh::tuple_from_id(const PrimitiveType type, const long gid) const
{
    if (type != PrimitiveType::Vertex) {
        throw std::runtime_error("Tuple switch: Invalid primitive type");
    }
    return vertex_tuple_from_id(gid);
}
} // namespace wmtk
