#pragma once

#include <memory>
#include <string_view>

namespace wmtk {
class Mesh;
namespace attribute {
    class MeshAttributeHandle;

}
} // namespace wmtk
namespace wmtk::components::multimesh {

// returns the new root mesh (the input mesh becomes a child mesh)
std::shared_ptr<Mesh> vertex_fusion(const attribute::MeshAttributeHandle& m, const std::string_view& name, double eps = 0.0);

} // namespace wmtk::components::multimesh
