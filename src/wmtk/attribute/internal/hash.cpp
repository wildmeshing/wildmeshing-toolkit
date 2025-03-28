#include "hash.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
std::size_t wmtk::hash<wmtk::attribute::AttributeHandle>::operator()(
    const wmtk::attribute::AttributeHandle& handle) const noexcept
{
    return handle.index();
    // return std::hash<int64_t>{}(handle.index);
}

size_t wmtk::hash<wmtk::attribute::MeshAttributeHandle>::operator()(
    const wmtk::attribute::MeshAttributeHandle& handle) const noexcept
{
    std::vector<size_t> data;
    data.emplace_back(handle_hash(handle));
    data.emplace_back(mesh_hash(handle));
    data.emplace_back(size_t(handle.held_type()));
    const size_t r = wmtk::utils::vector_hash(data);
    return r;
}
size_t wmtk::hash<wmtk::attribute::MeshAttributeHandle>::handle_hash(
    const wmtk::attribute::MeshAttributeHandle& handle) const noexcept
{
    return std::visit(
        [&](const auto& h) noexcept -> size_t {
            using T = std::decay_t<decltype(h)>;
            return hash<T>{}(h);
        },
        handle.m_handle);
}
size_t wmtk::hash<wmtk::attribute::MeshAttributeHandle>::mesh_hash(
    const wmtk::attribute::MeshAttributeHandle& handle) const noexcept
{
    if (!handle.is_valid()) {
        return -1; // TODO: this assumes the vector hash never returns a value of -1
    }
    // here we hash off of the absolute mesh id rather than the mesh itself to prevent cyclic
    // hashing
    return wmtk::utils::vector_hash(handle.mesh().absolute_multi_mesh_id());
}
