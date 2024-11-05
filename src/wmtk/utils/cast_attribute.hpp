#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/operations/attribute_update/CastAttributeTransferStrategy.hpp>

namespace wmtk::utils {

template <typename T>
void cast_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    const wmtk::attribute::MeshAttributeHandle& new_handle)
{
    std::visit(
        [&](const auto& typed_handle) noexcept {
            using ParentHandleType = std::decay_t<decltype(typed_handle)>;
            using ParentType = typename ParentHandleType::Type;

            wmtk::operations::attribute_update::CastAttributeTransferStrategy<T, ParentType> caster(
                new_handle,
                original_handle);
            caster.run_on_all();
        },
        original_handle.handle());
}
template <typename T>
wmtk::attribute::MeshAttributeHandle cast_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    Mesh& m,
    const std::string& new_attribute_name)
{
    auto new_handle = m.register_attribute<T>(
        new_attribute_name,
        original_handle.primitive_type(),
        original_handle.dimension());

    cast_attribute<T>(original_handle, new_handle);
    return new_handle;
}
} // namespace wmtk::utils
