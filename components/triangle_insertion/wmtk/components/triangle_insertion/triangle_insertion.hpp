#pragma once

#include <string>
#include <wmtk/Mesh.hpp>

namespace wmtk {
namespace components {

std::vector<std::pair<std::shared_ptr<wmtk::Mesh>, std::string>> triangle_insertion(
    const std::shared_ptr<Mesh>& mesh_in,
    const std::string& in_position,
    const std::shared_ptr<Mesh>& bg_mesh,
    const std::string& bg_position,
    std::vector<attribute::MeshAttributeHandle>& pass_through,
    bool make_child_free = false);

} // namespace components
} // namespace wmtk