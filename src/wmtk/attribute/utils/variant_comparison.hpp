#pragma once

namespace wmtk::attribute::utils {

template <typename T>
bool variant_comparison(const MeshAttributeHandle<T>& a, const MeshAttributeHandleVariant& var)
{
    return std::visit(
        [&](const auto& v) noexcept {
            if constexpr (std::is_same_v<MeshAttributeHandle<T>, std::decay_t<decltype(v)>>) {
                return v == a;

            } else {
                return false;
            }
        },
        var);
}

} // namespace wmtk::attribute::utils
