#pragma once

#include <memory>
#include <optional>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk {
class Mesh;
namespace attribute {
class MeshAttributeHandle;
}
} // namespace wmtk
namespace wmtk::components::multimesh {

struct TaggedRegion
{
    wmtk::attribute::MeshAttributeHandle handle;
    // tagged region is where == this value
    wmtk::attribute::MeshAttributeHandle::ValueVariant value;
};
struct FromTagOptions
{
    TaggedRegion mesh;
    std::optional<TaggedRegion> boundary;
    std::vector<wmtk::attribute::MeshAttributeHandle> passed_attributes;
};

// returns the child mesh created
std::shared_ptr<Mesh> from_tag(
    const wmtk::attribute::MeshAttributeHandle& handle,
    const wmtk::attribute::MeshAttributeHandle::ValueVariant& tag_value,
    const std::vector<wmtk::attribute::MeshAttributeHandle>& passed_attributes);

std::shared_ptr<Mesh> from_tag(const FromTagOptions& options);
} // namespace wmtk::components::multimesh
