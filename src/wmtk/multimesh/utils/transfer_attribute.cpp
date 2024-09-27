
#include "transfer_attribute.hpp"
#include <string>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/cast_attribute.hpp>

namespace wmtk::multimesh::utils {
wmtk::attribute::MeshAttributeHandle transfer_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    Mesh& m)
{
    const std::string name = original_handle.mesh().get_attribute_name(original_handle.handle());
    return transfer_attribute(original_handle, m, name);
}

wmtk::attribute::MeshAttributeHandle transfer_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    Mesh& m,
    const std::string& new_attribute_name)
{
    return std::visit(
        [&](const auto& h) {
            using T = typename std::decay_t<decltype(h)>::Type;
            return wmtk::utils::cast_attribute<T>(original_handle, m, new_attribute_name);
        },

        original_handle.handle());
}
} // namespace wmtk::multimesh::utils
