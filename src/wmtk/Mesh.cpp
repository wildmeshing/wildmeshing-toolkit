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


PrimitiveType Mesh::top_simplex_type() const
{
    int64_t dimension = top_cell_dimension();
    assert(dimension >= 0);
    assert(dimension < 4);
    return static_cast<PrimitiveType>(dimension);
}


std::vector<Tuple> Mesh::get_all(PrimitiveType type) const
{
    return get_all(type, false);
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


bool Mesh::is_hash_valid(const Tuple& tuple, const attribute::Accessor<int64_t>& hash_accessor) const
{
    const int64_t cid = tuple.m_global_cid;

    const int64_t desired_hash = get_cell_hash(cid, hash_accessor);
    if (tuple.m_hash != desired_hash) {
        logger().debug("Hash is not valid: {} != {}", tuple.m_hash, desired_hash);
        return false;
    }
    return true;
}

bool Mesh::is_valid_slow(const Tuple& tuple) const
{
    const attribute::Accessor<int64_t> hash_accessor = get_const_cell_hash_accessor();
    return is_valid(tuple, hash_accessor);
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

const attribute::Accessor<int64_t> Mesh::get_const_cell_hash_accessor() const
{
    return create_const_accessor(m_cell_hash_handle);
}

const attribute::Accessor<int64_t> Mesh::get_cell_hash_accessor() const
{
    return get_const_cell_hash_accessor();
}
attribute::Accessor<int64_t> Mesh::get_cell_hash_accessor()
{
    return create_accessor(m_cell_hash_handle);
}

void Mesh::update_cell_hash(const Tuple& cell, attribute::Accessor<int64_t>& hash_accessor)
{
    const int64_t cid = cell.m_global_cid;
    update_cell_hash(cid, hash_accessor);
}
void Mesh::update_cell_hash(const int64_t cid, attribute::Accessor<int64_t>& hash_accessor)
{
    auto& h = hash_accessor.index_access().scalar_attribute(cid);
    h = (h + 1) % (1 << 6);
}

void Mesh::update_cell_hashes(const std::vector<Tuple>& cells, attribute::Accessor<int64_t>& hash_accessor)
{
    for (const Tuple& t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}
void Mesh::update_cell_hashes(const std::vector<int64_t>& cells, attribute::Accessor<int64_t>& hash_accessor)
{
    for (const int64_t t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}

void Mesh::update_cell_hashes_slow(const std::vector<Tuple>& cells)
{
    attribute::Accessor<int64_t> hash_accessor = get_cell_hash_accessor();
    update_cell_hashes(cells, hash_accessor);
}


Tuple Mesh::resurrect_tuple(const Tuple& tuple, const attribute::Accessor<int64_t>& hash_accessor) const
{
    Tuple t = tuple;
    t.m_hash = get_cell_hash(tuple.m_global_cid, hash_accessor);
    return t;
}

Tuple Mesh::resurrect_tuple_slow(const Tuple& tuple)
{
    attribute::Accessor<int64_t> hash_accessor = get_cell_hash_accessor();
    return resurrect_tuple(tuple, hash_accessor);
}

int64_t Mesh::get_cell_hash(int64_t cell_index, const attribute::Accessor<int64_t>& hash_accessor) const
{
    return hash_accessor.index_access().const_scalar_attribute(cell_index);
}

int64_t Mesh::get_cell_hash_slow(int64_t cell_index) const
{
    const attribute::Accessor<int64_t> hash_accessor = get_cell_hash_accessor();
    return get_cell_hash(cell_index, hash_accessor);
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


void Mesh::update_vertex_operation_hashes(const Tuple& vertex, attribute::Accessor<int64_t>& hash_accessor)
{
    MultiMeshManager::update_vertex_operation_hashes_internal(*this, vertex, hash_accessor);
}

void Mesh::assert_capacity_valid() const
{
    m_attribute_manager.assert_capacity_valid();
}

int64_t Mesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (top_simplex_type()) {
    case PrimitiveType::Vertex: return static_cast<const PointMesh*>(this)->id(tuple, type);
    case PrimitiveType::Edge: return static_cast<const EdgeMesh*>(this)->id(tuple, type);
    case PrimitiveType::Triangle: return static_cast<const TriMesh*>(this)->id(tuple, type);
    case PrimitiveType::Tetrahedron: return static_cast<const TetMesh*>(this)->id(tuple, type);
    default: assert(false); return -1;
    }
}

} // namespace wmtk
