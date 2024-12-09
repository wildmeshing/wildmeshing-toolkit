#pragma once

#include <memory>
#include <vector>

namespace wmtk {
class Mesh;
} // namespace wmtk
namespace wmtk::components::multimesh {

// returns the new root mesh (the input mesh becomes a child mesh)
std::shared_ptr<Mesh> vertex_fusion(Mesh& m, double eps = 0.0);

} // namespace wmtk::components::multimesh
