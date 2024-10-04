#include "Mesh.hpp"
#include <numeric>

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/vector_hash.hpp>

#include "EdgeMesh.hpp"
#include "PointMesh.hpp"
#include "TetMesh.hpp"
#include "TriMesh.hpp"

#include "Primitive.hpp"

namespace wmtk {


std::vector<Tuple> Mesh::get_all(PrimitiveType type) const
{
    return get_all(type, false);
}

simplex::IdSimplex Mesh::get_id_simplex(const Tuple& tuple, PrimitiveType pt) const
{
    return simplex::IdSimplex(pt, id(tuple, pt));
}


std::vector<Tuple> Mesh::get_all(PrimitiveType type, const bool include_deleted) const
{
    std::vector<Tuple> ret;

    if (static_cast<int8_t>(type) > top_cell_dimension()) return ret;

    const int64_t cap = capacity(type);

    const attribute::Accessor<char> flag_accessor = get_flag_accessor(type);
    const attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();
    ret.reserve(cap);
    for (size_t index = 0; index < cap; ++index) {
        if (flag_accessor_indices.const_scalar_attribute(index) & 1)
            ret.emplace_back(tuple_from_id(type, index));
        else if (include_deleted)
            ret.emplace_back();
    }
    return ret;
}

void Mesh::serialize(MeshWriter& writer, const Mesh* local_root) const
{
    if (local_root == nullptr) {
        writer.write_absolute_id(m_multi_mesh_manager.absolute_id());
    } else {
        writer.write_absolute_id(m_multi_mesh_manager.relative_id(*this, *local_root));
    }
    writer.write_top_simplex_type(top_simplex_type());
    m_attribute_manager.serialize(writer);

    m_multi_mesh_manager.serialize(writer, local_root);
}


bool Mesh::is_boundary(const simplex::Simplex& s) const
{
    return is_boundary(s.primitive_type(), s.tuple());
}


bool Mesh::is_valid(const Tuple& tuple) const
{
    return !tuple.is_null() && !is_removed(tuple);
}

bool Mesh::is_removed(const Tuple& tuple) const
{
    return is_removed(tuple.m_global_cid);
}
bool Mesh::is_removed(const Tuple& t, PrimitiveType pt) const
{
    if (!is_removed(t)) {
        return is_removed(id(t, pt), pt);
    } else {
        return false;
    }
}
simplex::NavigatableSimplex Mesh::simplex_from_id(const PrimitiveType pt, const int64_t gid) const
{
    return simplex::NavigatableSimplex(pt, tuple_from_id(pt, gid), gid);
}
bool Mesh::is_removed(int64_t index) const
{
    return is_removed(index, top_simplex_type());
}
bool Mesh::is_removed(int64_t index, PrimitiveType pt) const
{
    const auto& flag_accessor = get_const_flag_accessor(pt);
    return !(flag_accessor.index_access().const_scalar_attribute(index) & 0x1);
}

bool Mesh::is_valid(const simplex::Simplex& s) const
{
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
    if (!is_valid(s.tuple())) {
        return false;
    } else {
        const int64_t id_tuple = id(s.tuple(), s.primitive_type());
        return id_tuple == s.m_index;
    }
#else
    return is_valid(s.tuple()) && !is_removed(s.tuple(), s.primitive_type());
#endif
}


const attribute::Accessor<char> Mesh::get_flag_accessor(PrimitiveType type) const
{
    return get_const_flag_accessor(type);
}
const attribute::Accessor<char> Mesh::get_const_flag_accessor(PrimitiveType type) const
{
    return create_const_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}
attribute::Accessor<char> Mesh::get_flag_accessor(PrimitiveType type)
{
    return create_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}


void Mesh::set_capacities_from_flags()
{
    for (int64_t dim = 0; dim < m_attribute_manager.m_capacities.size(); ++dim) {
        attribute::Accessor<char> flag_accessor = create_accessor<char>(m_flag_handles[dim]);
        m_attribute_manager.m_capacities[dim] = flag_accessor.reserved_size();
    }
}

bool Mesh::operator==(const Mesh& other) const
{
    return m_attribute_manager == other.m_attribute_manager;
}


std::vector<std::vector<int64_t>> Mesh::simplices_to_gids(
    const std::vector<std::vector<simplex::Simplex>>& simplices) const
{
    std::vector<std::vector<int64_t>> gids;
    gids.resize(simplices.size());
    for (int i = 0; i < simplices.size(); ++i) {
        auto simplices_i = simplices[i];
        for (const auto& simplex : simplices_i) {
            int64_t d = get_primitive_type_id(simplex.primitive_type());
            assert(d < 3);
            gids[d].emplace_back(id(simplex.tuple(), simplex.primitive_type()));
        }
    }
    return gids;
}


Tuple Mesh::switch_tuples(
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence) const
{
    return switch_tuples<std::initializer_list<PrimitiveType>>(tuple, op_sequence);
}
Tuple Mesh::switch_tuples_unsafe(
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence) const
{
    return switch_tuples_unsafe<std::initializer_list<PrimitiveType>>(tuple, op_sequence);
}


void Mesh::assert_capacity_valid() const
{
    m_attribute_manager.assert_capacity_valid();
}

} // namespace wmtk
