#include "Mesh.hpp"

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(const long& dimension)
{
    m_char_attributes.resize(dimension);
    m_long_attributes.resize(dimension);
    m_double_attributes.resize(dimension);
    m_capacities.resize(dimension, 0);
}

Mesh::~Mesh() = default;

template <typename T>
MeshAttributeHandle<T>
Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size)
{
    // return MeshAttributeHandle<T>{
    //    .m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    //    .m_primitive_type = ptype};

    MeshAttributeHandle<T> r;
    r.m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
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


// TODO
bool Mesh::is_valid(const Tuple& tuple) const
{
    // condition 1: global cid stays in bound, and is not removed

    // condition 2: hash


    // Condition 3: local ids are consistent
    const int v = tuple.m_local_vid;
    switch (tuple.m_local_eid) {
    case 0:
        if (tuple.m_local_vid == 1 || tuple.m_local_vid == 2)
            return true;
        else
            return false;
    case 1:
        if (tuple.m_local_vid == 0 || tuple.m_local_vid == 2)
            return true;
        else
            return false;
    case 2:
        if (tuple.m_local_vid == 1 || tuple.m_local_vid == 0)
            return true;
        else
            return false;
    default: throw std::runtime_error("tuple invlid failed local ids check");
    }
}


template MeshAttributeHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long);
template MeshAttributeHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long);
template MeshAttributeHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long);

} // namespace wmtk
