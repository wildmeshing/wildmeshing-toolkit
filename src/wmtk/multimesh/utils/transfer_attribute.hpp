#pragma once
#include <string>
namespace wmtk {
class Mesh;
namespace attribute {
class MeshAttributeHandle;
}
} // namespace wmtk

namespace wmtk::multimesh::utils {

wmtk::attribute::MeshAttributeHandle transfer_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    Mesh& m);
wmtk::attribute::MeshAttributeHandle transfer_attribute(
    const wmtk::attribute::MeshAttributeHandle& original_handle,
    Mesh& m,
    const std::string& new_attribute_name);
} // namespace wmtk::multimesh::utils
