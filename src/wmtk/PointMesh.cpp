#include "PointMesh.hpp"

namespace wmtk {
Tuple PointMesh::vertex_tuple_from_id(int64_t id) const
{
    return Tuple(-1, -1, -1, id, get_cell_hash_slow(id));
}

PointMesh::PointMesh()
    : MeshCRTP<PointMesh>(0)
{}


PointMesh::PointMesh(int64_t size)
    : PointMesh()
{
    initialize(size);
}

Tuple PointMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    throw std::runtime_error("Tuple switch: Invalid primitive type");
}
bool PointMesh::is_ccw(const Tuple&) const
{
    // trivial orientation so nothing can happen
    return true;
}
bool PointMesh::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: break;
    }
    assert(
        false); // "tried to compute the boundary of a point mesh for an invalid simplex dimension"
    // every point is on the interior as it has no boundary simplices
    return false;
}

bool PointMesh::is_boundary_vertex(const Tuple&) const
{
    // every point is on the interior as it has no boundary simplices
    return false;
}

void PointMesh::initialize(int64_t count)
{
    set_capacities({count});
    reserve_attributes_to_fit();
    attribute::Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        v_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}


bool PointMesh::is_valid(const Tuple& tuple, const attribute::Accessor<int64_t>& hash_accessor)
    const
{
    if (tuple.is_null()) return false;
#if defined(WMTK_ENABLE_HASH_UPDATE)
    return Mesh::is_hash_valid(tuple, hash_accessor);
#else
    const auto& flag_accessor = get_const_flag_accessor(PrimitiveType::Vertex);
    return flag_accessor.index_access().const_scalar_attribute(tuple.m_global_cid) & 0x1;
#endif
}


Tuple PointMesh::tuple_from_id(const PrimitiveType type, const int64_t gid) const
{
    assert(type == PrimitiveType::Vertex); // "Tuple switch: Invalid primitive type"
    return vertex_tuple_from_id(gid);
}

std::vector<std::vector<TypedAttributeHandle<int64_t>>> PointMesh::connectivity_attributes() const
{
    std::vector<std::vector<TypedAttributeHandle<int64_t>>> handles(0);

    return handles;
}

} // namespace wmtk
