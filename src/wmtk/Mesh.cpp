#include "Mesh.hpp"
#include <numeric>

#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(const long& dimension)
    : m_char_attributes(dimension + 1)
    , m_long_attributes(dimension + 1)
    , m_double_attributes(dimension + 1)
    , m_capacities(dimension + 1, 0)
    , m_cell_hash_handle(register_attribute<long>("hash", static_cast<PrimitiveType>(dimension), 1))
{
    m_flag_handles.reserve(dimension + 1);
    for (long j = 0; j <= dimension; ++j) {
        m_flag_handles.emplace_back(
            register_attribute<char>("flags", static_cast<PrimitiveType>(j), 1));
    }
}

Mesh::~Mesh() = default;

std::vector<Tuple> Mesh::get_all(const PrimitiveType& type) const
{
    ConstAccessor<char> flag_accessor = get_flag_accessor(type);
    std::vector<Tuple> ret;
    long cap = capacity(type);
    ret.reserve(cap);
    for (size_t index = 0; index < cap; ++index) {
        if (!(flag_accessor.scalar_attribute(index) & 1)) {
            ret.emplace_back(tuple_from_id(type, index));
        }
    }
    return ret;
}

void Mesh::serialize(MeshWriter& writer)
{
    for (long dim = 0; dim < m_capacities.size(); ++dim) {
        if (!writer.write(dim)) continue;
        m_char_attributes[dim].serialize(dim, writer);
        m_long_attributes[dim].serialize(dim, writer);
        m_double_attributes[dim].serialize(dim, writer);
    }
}

template <typename T>
MeshAttributeHandle<T>
Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size, bool replace)
{
    // return MeshAttributeHandle<T>{
    //    .m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    //    .m_primitive_type = ptype};

    MeshAttributeHandle<T> r;
    r.m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size, replace),
    r.m_primitive_type = ptype;
    return r;
}

std::vector<long> Mesh::request_simplex_indices(PrimitiveType type, long count)
{
    // passses back a set of new consecutive ids. in hte future this could do
    // something smarter for re-use but that's probably too much work
    long current_capacity = capacity(type);

    // enable newly requested simplices
    Accessor<char> flag_accessor = get_flag_accessor(type);
    long max_size = flag_accessor.size();

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
    size_t simplex_dim = get_simplex_dimension(type);

    m_capacities[simplex_dim] = new_capacity;


    for (const long simplex_index : ret) {
        flag_accessor.scalar_attribute(simplex_index) |= 0x1;
    }

    return ret;
}

long Mesh::capacity(PrimitiveType type) const
{
    return m_capacities.at(get_simplex_dimension(type));
}

bool Mesh::simplex_is_equal(const Simplex& s0, const Simplex& s1) const
{
    return (s0.primitive_type() == s1.primitive_type()) && (id(s0) == id(s1));
}

bool Mesh::simplex_is_less(const Simplex& s0, const Simplex& s1) const
{
    if (s0.primitive_type() < s1.primitive_type()) {
        return true;
    }
    if (s0.primitive_type() > s1.primitive_type()) {
        return false;
    }
    return id(s0) < id(s1);
}

void Mesh::reserve_attributes_to_fit()
{
    for (long dim = 0; dim < m_capacities.size(); ++dim) {
        const long capacity = m_capacities[dim];
        reserve_attributes(dim, capacity);
    }
}
void Mesh::reserve_attributes(PrimitiveType type, long size)
{
    reserve_attributes(get_simplex_dimension(type), size);
}
void Mesh::reserve_attributes(long dimension, long capacity)
{
    m_char_attributes[dimension].reserve(capacity);
    m_long_attributes[dimension].reserve(capacity);
    m_double_attributes[dimension].reserve(capacity);
}
void Mesh::set_capacities(std::vector<long> capacities)
{
    assert(capacities.size() == m_capacities.size());
    m_capacities = std::move(capacities);
    reserve_attributes_to_fit();
}
ConstAccessor<char> Mesh::get_flag_accessor(PrimitiveType type) const
{
    return create_accessor(m_flag_handles.at(get_simplex_dimension(type)));
}
Accessor<char> Mesh::get_flag_accessor(PrimitiveType type)
{
    return create_accessor(m_flag_handles.at(get_simplex_dimension(type)));
}

ConstAccessor<long> Mesh::get_cell_hash_accessor() const
{
    return create_accessor(m_cell_hash_handle);
}
Accessor<long> Mesh::get_cell_hash_accessor()
{
    return create_accessor(m_cell_hash_handle);
}

long Mesh::get_cell_hash_slow(long cell_index) const
{
    ConstAccessor<long> hash_accessor = get_cell_hash_accessor();
    return hash_accessor.scalar_attribute(cell_index);
}

void Mesh::set_capacities_from_flags()
{
    for (const auto& flag_handle : m_flag_handles) {
        auto fa = create_const_accessor(flag_handle);
    }
    // for(long

    throw "not implemented";
}

bool Mesh::operator==(const Mesh& other) const
{
    return m_capacities == other.m_capacities && m_char_attributes == other.m_char_attributes &&
           m_long_attributes == other.m_long_attributes &&
           m_double_attributes == other.m_double_attributes;
}


std::vector<std::vector<long>> Mesh::simplices_to_gids(
    const std::vector<std::vector<Simplex>>& simplices) const
{
    std::vector<std::vector<long>> gids;
    gids.resize(simplices.size());
    for (int i = 0; i < simplices.size(); ++i) {
        auto simplices_i = simplices[i];
        for (auto simplex : simplices_i) {
            long d = get_simplex_dimension(simplex.primitive_type());
            assert(d < 3);
            gids[d].emplace_back(id(simplex.tuple(), simplex.primitive_type()));
        }
    }
    return gids;
}

template MeshAttributeHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);
template MeshAttributeHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);
template MeshAttributeHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);

} // namespace wmtk
