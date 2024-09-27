#pragma once

#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk {
class Mesh;
namespace attribute {
class MeshAttributeHandle;
}
} // namespace wmtk
namespace wmtk::components::multimesh {

void from_boundary(
    Mesh& m,
    const PrimitiveType boundary_dimension,
    const std::string& attribute_name,
    char value = 1,
    const std::vector<wmtk::attribute::MeshAttributeHandle>& passed_attributes = {});
} // namespace wmtk::components::multimesh
