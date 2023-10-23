#include "Mesh.hpp"
#include <numeric>

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(Mesh&& other) = default;
Mesh::Mesh(const Mesh& other) = default;
Mesh& Mesh::operator=(const Mesh& other) = default;
Mesh& Mesh::operator=(Mesh&& other) = default;
Mesh::Mesh(const long& dimension)
    : Mesh(dimension, dimension, get_primitive_type_from_id(dimension))
{}

Mesh::Mesh(const long& dimension, const long& max_primitive_type_id, PrimitiveType hash_type)
    : m_attribute_manager(max_primitive_type_id + 1)
    , m_cell_hash_handle(register_attribute<long>("hash", hash_type, 1))
{
    m_flag_handles.reserve(max_primitive_type_id + 1);
    for (long j = 0; j <= max_primitive_type_id; ++j) {
        m_flag_handles.emplace_back(
            register_attribute<char>("flags", get_primitive_type_from_id(j), 1));
    }
}

Mesh::~Mesh() = default;

PrimitiveType Mesh::top_simplex_type() const
{
    long dimension = top_cell_dimension();
    assert(dimension >= 0);
    assert(dimension < 4);
    return static_cast<PrimitiveType>(dimension);
}

std::vector<Tuple> Mesh::get_all(PrimitiveType type) const
{
    ConstAccessor<char> flag_accessor = get_flag_accessor(type);
    const attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();
    std::vector<Tuple> ret;
    long cap = capacity(type);
    ret.reserve(cap);
    for (size_t index = 0; index < cap; ++index) {
        if ((flag_accessor_indices.const_scalar_attribute(index) & 1)) {
            ret.emplace_back(tuple_from_id(type, index));
        }
    }
    return ret;
}

void Mesh::serialize(MeshWriter& writer)
{
    m_attribute_manager.serialize(writer);
}

template <typename T>
MeshAttributeHandle<T> Mesh::register_attribute(
    const std::string& name,
    PrimitiveType ptype,
    long size,
    bool replace,
    T default_value)
{
    return m_attribute_manager.register_attribute<T>(name, ptype, size, replace, default_value);
}

std::vector<long> Mesh::request_simplex_indices(PrimitiveType type, long count)
{
    // passses back a set of new consecutive ids. in hte future this could do
    // something smarter for re-use but that's probably too much work
    long current_capacity = capacity(type);

    // enable newly requested simplices
    Accessor<char> flag_accessor = get_flag_accessor(type);
    long max_size = flag_accessor.reserved_size();

    if (current_capacity + count > max_size) {
        logger().warn(
            "Requested more {} simplices than available (have {}, wanted {}, can only have at most "
            "{}",
            primitive_type_name(type),
            current_capacity,
            count,
            max_size);
        return {};
    }

    std::vector<long> ret(count);
    std::iota(ret.begin(), ret.end(), current_capacity);


    long new_capacity = ret.back() + 1;
    size_t primitive_id = get_primitive_type_id(type);

    m_attribute_manager.m_capacities[primitive_id] = new_capacity;

    attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();

    for (const long simplex_index : ret) {
        flag_accessor_indices.scalar_attribute(simplex_index) |= 0x1;
    }

    return ret;
}

long Mesh::capacity(PrimitiveType type) const
{
    return m_attribute_manager.m_capacities.at(get_primitive_type_id(type));
}

bool Mesh::is_hash_valid(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const
{
    const long cid = tuple.m_global_cid;
    return tuple.m_hash == get_cell_hash(cid, hash_accessor);
}

bool Mesh::is_valid_slow(const Tuple& tuple) const
{
    ConstAccessor<long> hash_accessor = get_const_cell_hash_accessor();
    return is_valid(tuple, hash_accessor);
}

bool Mesh::simplices_are_equal(const Simplex& s0, const Simplex& s1) const
{
    return (s0.primitive_type() == s1.primitive_type()) && (id(s0) == id(s1));
}

bool Mesh::simplex_is_less(const Simplex& s0, const Simplex& s1) const
{
    if (s0.primitive_type() == s1.primitive_type()) {
        return id(s0) < id(s1);
    }
    return s0.primitive_type() < s1.primitive_type();
}

void Mesh::reserve_attributes_to_fit()
{
    m_attribute_manager.reserve_to_fit();
}
void Mesh::reserve_attributes(PrimitiveType type, long size)
{
    m_attribute_manager.reserve_attributes(get_primitive_type_id(type), size);
}
void Mesh::set_capacities(std::vector<long> capacities)
{
    m_attribute_manager.set_capacities(std::move(capacities));
}
ConstAccessor<char> Mesh::get_flag_accessor(PrimitiveType type) const
{
    return get_const_flag_accessor(type);
}
ConstAccessor<char> Mesh::get_const_flag_accessor(PrimitiveType type) const
{
    return create_const_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}
Accessor<char> Mesh::get_flag_accessor(PrimitiveType type)
{
    return create_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}

ConstAccessor<long> Mesh::get_const_cell_hash_accessor() const
{
    return create_const_accessor(m_cell_hash_handle);
}

ConstAccessor<long> Mesh::get_cell_hash_accessor() const
{
    return get_const_cell_hash_accessor();
}
Accessor<long> Mesh::get_cell_hash_accessor()
{
    return create_accessor(m_cell_hash_handle);
}

void Mesh::update_cell_hash(const Tuple& cell, Accessor<long>& hash_accessor)
{
    const long cid = cell.m_global_cid;
    update_cell_hash(cid, hash_accessor);
}
void Mesh::update_cell_hash(const long cid, Accessor<long>& hash_accessor)
{
    ++hash_accessor.index_access().scalar_attribute(cid);
}

void Mesh::update_cell_hashes(const std::vector<Tuple>& cells, Accessor<long>& hash_accessor)
{
    for (const Tuple& t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}
void Mesh::update_cell_hashes(const std::vector<long>& cells, Accessor<long>& hash_accessor)
{
    for (const long t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}

void Mesh::update_cell_hashes_slow(const std::vector<Tuple>& cells)
{
    Accessor<long> hash_accessor = get_cell_hash_accessor();
    update_cell_hashes(cells, hash_accessor);
}


Tuple Mesh::resurrect_tuple(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const
{
    Tuple t = tuple;
    t.m_hash = get_cell_hash(tuple.m_global_cid, hash_accessor);
    return t;
}

Tuple Mesh::resurrect_tuple_slow(const Tuple& tuple)
{
    Accessor<long> hash_accessor = get_cell_hash_accessor();
    return resurrect_tuple(tuple, hash_accessor);
}

long Mesh::get_cell_hash(long cell_index, const ConstAccessor<long>& hash_accessor) const
{
    return hash_accessor.index_access().const_scalar_attribute(cell_index);
}

long Mesh::get_cell_hash_slow(long cell_index) const
{
    ConstAccessor<long> hash_accessor = get_cell_hash_accessor();
    return get_cell_hash(cell_index, hash_accessor);
}

void Mesh::set_capacities_from_flags()
{
    for (long dim = 0; dim < m_attribute_manager.m_capacities.size(); ++dim) {
        Accessor<char> flag_accessor = create_accessor<char>(m_flag_handles[dim]);
        m_attribute_manager.m_capacities[dim] = flag_accessor.reserved_size();
    }
}

bool Mesh::operator==(const Mesh& other) const
{
    return m_attribute_manager == other.m_attribute_manager;
}


std::vector<std::vector<long>> Mesh::simplices_to_gids(
    const std::vector<std::vector<Simplex>>& simplices) const
{
    std::vector<std::vector<long>> gids;
    gids.resize(simplices.size());
    for (int i = 0; i < simplices.size(); ++i) {
        auto simplices_i = simplices[i];
        for (auto simplex : simplices_i) {
            long d = get_primitive_type_id(simplex.primitive_type());
            assert(d < 3);
            gids[d].emplace_back(id(simplex.tuple(), simplex.primitive_type()));
        }
    }
    return gids;
}

attribute::AttributeScopeHandle Mesh::create_scope()
{
    return m_attribute_manager.create_scope(*this);
}


template MeshAttributeHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, char);
template MeshAttributeHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, long);
template MeshAttributeHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, double);

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


std::vector<long> Mesh::absolute_multi_mesh_id() const
{
    return m_multi_mesh_manager.absolute_id();
}
void Mesh::register_child_mesh(
    const std::shared_ptr<Mesh>& child_mesh_ptr,
    const std::vector<std::array<Tuple, 2>>& map_tuples)
{
    m_multi_mesh_manager.register_child_mesh(*this, child_mesh_ptr, map_tuples);
}


std::vector<Simplex> Mesh::map(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map(*this, other_mesh, my_simplex);
}
Simplex Mesh::map_to_parent(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_parent(*this, my_simplex);
}
Simplex Mesh::map_to_root(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root(*this, my_simplex);
}
std::vector<Simplex> Mesh::map_to_child(const Mesh& child_mesh, const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_child(*this, child_mesh, my_simplex);
}

std::vector<Tuple> Mesh::map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_tuples(*this, other_mesh, my_simplex);
}
Tuple Mesh::map_to_parent_tuple(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_parent_tuple(*this, my_simplex);
}
Tuple Mesh::map_to_root_tuple(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root_tuple(*this, my_simplex);
}
std::vector<Tuple> Mesh::map_to_child_tuples(const Mesh& child_mesh, const Simplex& my_simplex)
    const
{
    return m_multi_mesh_manager.map_to_child_tuples(*this, child_mesh, my_simplex);
}

bool Mesh::is_multi_mesh_root() const
{
    return m_multi_mesh_manager.is_root();
}
Mesh& Mesh::get_multi_mesh_root()
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}
const Mesh& Mesh::get_multi_mesh_root() const
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}

// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(PrimitiveType type, long size)
{
    m_attribute_manager.reserve_more_attributes(get_simplex_dimension(type), size);
}
// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(const std::vector<long>& sizes)
{
    assert(top_cell_dimension() + 1 == sizes.size());
    for (long j = 0; j < sizes.size(); ++j) {
        m_attribute_manager.reserve_more_attributes(j, sizes[j]);
    }
}


} // namespace wmtk
