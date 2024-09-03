#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk {
namespace components {

std::shared_ptr<wmtk::TetMesh> triangle_insertion(
    const std::shared_ptr<Mesh>& mesh_in,
    const std::string& in_position,
    const std::shared_ptr<Mesh>& bg_mesh,
    const std::string& bg_position,
    bool make_child_free = false);

} // namespace components
} // namespace wmtk