
#include <numeric>
#include "Mesh.hpp"

#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh(Mesh&& other)
    : m_attribute_manager(std::move(other.m_attribute_manager))
{
    m_multi_mesh_manager = std::move(other.m_multi_mesh_manager);
    // m_attributes = std::move(other.m_attributes);
    m_flag_handles = std::move(other.m_flag_handles);
    m_cell_hash_handle = std::move(other.m_cell_hash_handle);

    // for (size_t i = 0; i < m_attributes.size(); ++i) {
    //     m_attributes[i] = std::visit(
    //         [&](auto&& h) -> attribute::MeshAttributeHandleVariant {
    //             return std::decay_t<decltype(h)>(*this, h);
    //         },
    //         m_attributes[i]);
    // }
}

Mesh::Mesh(const Mesh& other)
    : std::enable_shared_from_this<wmtk::Mesh>(other)
    , m_attribute_manager(other.m_attribute_manager)
{
    m_multi_mesh_manager = other.m_multi_mesh_manager;
    // m_attributes = other.m_attributes;
    m_flag_handles = other.m_flag_handles;
    m_cell_hash_handle = other.m_cell_hash_handle;

    // for (size_t i = 0; i < m_attributes.size(); ++i) {
    //     m_attributes[i] = std::visit(
    //         [&](auto&& h) -> attribute::MeshAttributeHandleVariant {
    //             return std::decay_t<decltype(h)>(*this, h);
    //         },
    //         m_attributes[i]);
    // }
}

Mesh& Mesh::operator=(const Mesh& other)
{
    m_attribute_manager = other.m_attribute_manager;
    m_multi_mesh_manager = other.m_multi_mesh_manager;
    // m_attributes = other.m_attributes;
    m_flag_handles = other.m_flag_handles;
    m_cell_hash_handle = other.m_cell_hash_handle;

    // for (size_t i = 0; i < m_attributes.size(); ++i) {
    //     m_attributes[i] = std::visit(
    //         [&](auto&& h) -> attribute::MeshAttributeHandleVariant {
    //             return std::decay_t<decltype(h)>(*this, h);
    //         },
    //         m_attributes[i]);
    // }

    return *this;
}

Mesh& Mesh::operator=(Mesh&& other)
{
    m_attribute_manager = std::move(other.m_attribute_manager);
    m_multi_mesh_manager = std::move(other.m_multi_mesh_manager);
    // m_attributes = std::move(other.m_attributes);
    m_flag_handles = std::move(other.m_flag_handles);
    m_cell_hash_handle = std::move(other.m_cell_hash_handle);

    // for (size_t i = 0; i < m_attributes.size(); ++i) {
    //     m_attributes[i] = std::visit(
    //         [&](auto&& h) -> attribute::MeshAttributeHandleVariant {
    //             return std::decay_t<decltype(h)>(*this, h);
    //         },
    //         m_attributes[i]);
    // }


    return *this;
}

Mesh::Mesh(const int64_t& dimension)
    : Mesh(dimension, dimension, get_primitive_type_from_id(dimension))
{}

Mesh::Mesh(const int64_t& dimension, const int64_t& max_primitive_type_id, PrimitiveType hash_type)
    : m_attribute_manager(max_primitive_type_id + 1)
    , m_cell_hash_handle(register_attribute_nomesh<int64_t>("hash", hash_type, 1))
{
    m_flag_handles.reserve(max_primitive_type_id + 1);
    for (int64_t j = 0; j <= max_primitive_type_id; ++j) {
        m_flag_handles.emplace_back(
            register_attribute_nomesh<char>("flags", get_primitive_type_from_id(j), 1));
    }
}


Mesh::~Mesh() = default;
} // namespace wmtk
