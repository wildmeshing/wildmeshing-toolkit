#pragma once
#include <string_view>
#include <array>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::components::multimesh {
class NamedMultiMesh;
class MeshCollection;
struct AttributeDescription;

//namespace detail {
//// turns an attribute path mesh.path/attrname to mesh.path attrname
//std::array<std::string_view, 2> decompose_attribute_path(
//    std::string_view attribute_path);
//std::array<std::string_view, 2> decompose_attribute_path(
//    const AttributeDescription& description);
//} // namespace detail

} // namespace wmtk::components::multimesh
