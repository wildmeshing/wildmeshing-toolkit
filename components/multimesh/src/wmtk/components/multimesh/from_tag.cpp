
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/transfer_attribute.hpp>


namespace wmtk::components::multimesh {

namespace {
    std::shared_ptr<Mesh> from_tag(
    wmtk::attribute::MeshAttributeHandle& handle,
    const wmtk::attribute::MeshAttributeHandle::ValueVariant& tag_value,
    const std::vector<wmtk::attribute::MeshAttributeHandle>& passed_attributes)
{
    auto child_mesh =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(handle, tag_value);

    for (const auto& attr : passed_attributes) {
        assert(attr.is_same_mesh(handle.mesh()));

        wmtk::multimesh::utils::transfer_attribute(attr, *child_mesh);
    }
    return child_mesh;
}
} // namespace

std::shared_ptr<Mesh> from_tag(
    const wmtk::attribute::MeshAttributeHandle& handle,

    const wmtk::attribute::MeshAttributeHandle::ValueVariant& tag_value,

    const std::vector<wmtk::attribute::MeshAttributeHandle>& passed_attributes)

{
    // constness is annoying, but want to let rvalues get passed in?
    wmtk::attribute::MeshAttributeHandle h = handle;
    return from_tag(h, tag_value, passed_attributes);
}

} // namespace wmtk::components::multimesh
