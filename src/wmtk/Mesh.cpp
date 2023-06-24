#include "Mesh.hpp"

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(const long& dimension)
    : m_char_attributes(dimension)
    , m_long_attributes(dimension)
    , m_double_attributes(dimension)
    , m_capacities(dimension, 0)
    , m_cell_hash_handle(
          register_attribute<long>("hash", static_cast<PrimitiveType>(dimension - 1), 1))
{
    m_flag_handles.reserve(dimension);
    for (long j = 0; j < dimension; ++j) {
        m_flag_handles.emplace_back(
            register_attribute<char>("flags", static_cast<PrimitiveType>(j), 1));
    }
}

Mesh::~Mesh() = default;

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

long Mesh::capacity(PrimitiveType type) const
{
    return m_capacities.at(get_simplex_dimension(type));
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

bool Mesh::operator==(const Mesh& other) const
{
    return m_capacities == other.m_capacities && m_char_attributes == other.m_char_attributes &&
           m_long_attributes == other.m_long_attributes &&
           m_double_attributes == other.m_double_attributes;
}

template MeshAttributeHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);
template MeshAttributeHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);
template MeshAttributeHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool);

} // namespace wmtk
