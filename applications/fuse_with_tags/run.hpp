#pragma once

#include <string_view>
namespace wmtk {
class Mesh;
namespace components::multimesh {
class MeshCollection;
class NamedMultiMesh;
} // namespace components::multimesh
} // namespace wmtk

wmtk::components::multimesh::NamedMultiMesh& run(
    wmtk::components::multimesh::MeshCollection& collection,
    const std::string_view& name,
    const std::string_view& tag_format,
    const std::string_view& position_attribute_name);
