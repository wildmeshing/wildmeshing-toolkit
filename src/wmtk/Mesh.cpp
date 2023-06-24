#include "Mesh.hpp"

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(const long& dimension)
{
    m_char_attributes.resize(dimension);
    m_long_attributes.resize(dimension);
    m_double_attributes.resize(dimension);
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
    switch (type) {
    case PrimitiveType::Vertex: return m_capacities[0]; break;
    case PrimitiveType::Edge: return m_capacities[1]; break;
    case PrimitiveType::Face: return m_capacities[2]; break;
    case PrimitiveType::Tetrahedron: {
        if (m_capacities.size() < 4)
            throw std::runtime_error("TetMesh not initialized");
        else
            return m_capacities[3];
        break;
    }
    default: throw std::runtime_error("Invalid primitive type");
    }
}

void Mesh::mesh_attributes_reserve(const PrimitiveType& top_d)
{
    for (long dim = 0; dim < m_capacities.size(); ++dim) {
        const long capacity = m_capacities[dim];
        m_char_attributes[dim].reserve(capacity);
        m_long_attributes[dim].reserve(capacity);
        m_double_attributes[dim].reserve(capacity);
        // m_rational_attributes[get_simplex_dimension(d)].reserve(capacity);
    }
}

template MeshAttributeHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long);
template MeshAttributeHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long);
template MeshAttributeHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long);

} // namespace wmtk
