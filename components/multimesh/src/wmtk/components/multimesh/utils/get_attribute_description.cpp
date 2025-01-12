

#include <wmtk/Mesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include "AttributeDescription.hpp"
namespace wmtk::components::multimesh {
namespace utils {
AttributeDescription get_attribute_handle(
    const MeshCollection& collection,
    const wmtk::attribute::MeshAttributeHandle& handle)
{
    return {};
}
AttributeDescription get_attribute_handle(
    const NamedMultiMesh& nmm,
    const wmtk::attribute::MeshAttributeHandle& handle)
{
    nmm.get_name(handle.mesh());
}
} // namespace utils
} // namespace wmtk::components::multimesh
