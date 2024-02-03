
#include <numeric>
#include "Mesh.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(Mesh&& other)
    : m_attribute_manager(std::move(other.m_attribute_manager))
    , m_multi_mesh_manager(std::move(other.m_multi_mesh_manager))
    , m_top_cell_dimension(other.m_top_cell_dimension)
{
    m_flag_handles = std::move(other.m_flag_handles);
    m_cell_hash_handle = std::move(other.m_cell_hash_handle);
}


Mesh& Mesh::operator=(Mesh&& other)
{
    m_attribute_manager = std::move(other.m_attribute_manager);
    m_multi_mesh_manager = std::move(other.m_multi_mesh_manager);
    m_flag_handles = std::move(other.m_flag_handles);
    m_top_cell_dimension = other.m_top_cell_dimension;
    m_cell_hash_handle = std::move(other.m_cell_hash_handle);

    return *this;
}

Mesh::Mesh(const int64_t& dimension)
    : Mesh(dimension, dimension, get_primitive_type_from_id(dimension))
{}

Mesh::Mesh(const int64_t& dimension, const int64_t& max_primitive_type_id, PrimitiveType hash_type)
    : m_attribute_manager(max_primitive_type_id + 1)
    , m_multi_mesh_manager(max_primitive_type_id + 1)
    , m_top_cell_dimension(dimension)
    , m_cell_hash_handle(register_attribute_typed<int64_t>("hash", hash_type, 1, false, 0))
{
    m_flag_handles.reserve(max_primitive_type_id + 1);
    for (int64_t j = 0; j <= max_primitive_type_id; ++j) {
        m_flag_handles.emplace_back(
            register_attribute_typed<char>("flags", get_primitive_type_from_id(j), 1, false, 0));
    }
}


Mesh::~Mesh() = default;
} // namespace wmtk
