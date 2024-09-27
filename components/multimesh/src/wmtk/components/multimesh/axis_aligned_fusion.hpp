#pragma once

#include <memory>
#include <vector>

namespace wmtk {
class Mesh;
} // namespace wmtk
namespace wmtk::components::multimesh {

// TODO: this really shouldn't require returning a mesh
// void axis_aligned_fusion(Mesh& m, const std::vector<bool>& axes_to_fuse, double eps = 1e-10);
std::shared_ptr<Mesh>
axis_aligned_fusion(const Mesh& m, const std::vector<bool>& axes_to_fuse, double eps = 1e-10);

} // namespace wmtk::components::multimesh
