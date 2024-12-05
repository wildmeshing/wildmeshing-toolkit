#pragma once
#include "AttributeDescription.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"

namespace wmtk {
class Mesh;
namespace attribute {
class MeshAttributeHandle;
}
} // namespace wmtk

namespace wmtk::components::multimesh {
class MeshCollection;
class NamedMultiMesh;
namespace utils {
struct AttributeDescription;

AttributeDescription get_attribute_handle(
    const MeshCollection& collection,
    const wmtk::attribute::MeshAttributeHandle& handle);
AttributeDescription get_attribute_handle(
    const NamedMultiMesh& collection,
    const wmtk::attribute::MeshAttributeHandle& handle);
} // namespace utils
} // namespace wmtk::components::multimesh
