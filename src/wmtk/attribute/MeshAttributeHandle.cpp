#include "MeshAttributeHandle.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
MeshAttributeHandle::MeshAttributeHandle(Mesh& m, const HandleVariant& h)
    : m_mesh(&m)
    , m_handle(h)
{}

bool MeshAttributeHandle::is_same_mesh(const Mesh& m) const
{
    assert(m_mesh != nullptr);
    return m_mesh == &m;
}

Mesh& MeshAttributeHandle::mesh()
{
    assert(m_mesh != nullptr);
    return *m_mesh;
}
const Mesh& MeshAttributeHandle::mesh() const
{
    assert(m_mesh != nullptr);
    return *m_mesh;
}
auto MeshAttributeHandle::held_type() const -> HeldType
{
    return std::visit(
        [](const auto& h) -> HeldType {
            using T = std::decay_t<decltype(h)>;
            return held_type_from_handle<T>();
        },
        m_handle);
}

bool MeshAttributeHandle::is_valid() const
{
    return m_mesh != nullptr &&
           std::visit(
               [](const auto& h) -> bool {
                   using T = std::decay_t<decltype(h)>;
                   if constexpr (held_type_from_handle<T>() == HeldType::HybridRational) {
                       return h.get_char().is_valid();
                   } else {
                       return h.is_valid();
                   }
               },
               m_handle);
}

int64_t MeshAttributeHandle::dimension() const
{
    return std::visit(
        [&](auto&& h) -> int64_t {
            using T = std::decay_t<decltype(h)>;
            if constexpr (held_type_from_handle<T>() == HeldType::HybridRational) {
                return mesh().get_attribute_dimension(h.get_char());
            } else {
                return mesh().get_attribute_dimension(h);
            }
        },
        m_handle);
}

// std::string MeshAttributeHandle::name() const
//{
//     std::visit([&](auto&& h){return mesh().get_attribute_name(h);}, m_handle);
// }


// AttributeHandle MeshAttributeHandle::base_handle() const
//{
//     return std::visit([](const auto& h) { return h.m_base_handle; }, m_handle);
// }
} // namespace wmtk::attribute
