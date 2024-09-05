#pragma once
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::attribute::utils {

template <typename T>
bool variant_comparison(
    const TypedAttributeHandle<T>& a,
    const MeshAttributeHandle::HandleVariant& var)
{
    return std::visit(
        [&](const auto& v) noexcept {
            if constexpr (std::is_same_v<TypedAttributeHandle<T>, std::decay_t<decltype(v)>>) {
                return v == a;

            } else {
                return false;
            }
        },
        var);
}

} // namespace wmtk::attribute::utils
