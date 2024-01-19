#include "MeshAttributeHandle.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
auto MeshAttributeHandle::held_type() const -> HeldType
{
    return std::visit(
        [](const auto& h) -> HeldType {
            using T = std::decay_t<decltype(h)>;
            return held_type_from_primitive<typename T::value_type>();
        },
        m_handle);
}

bool MeshAttributeHandle::is_valid() const
{
    return m_mesh != nullptr &&
        std::visit([](const auto& h) -> bool { return h.is_valid(); }, m_handle);
}
//std::string MeshAttributeHandle::name() const
//{
//    std::visit([&](auto&& h){return mesh().get_attribute_name(h);}, m_handle);
//}


// AttributeHandle MeshAttributeHandle::base_handle() const
//{
//     return std::visit([](const auto& h) { return h.m_base_handle; }, m_handle);
// }
} // namespace wmtk::attribute
