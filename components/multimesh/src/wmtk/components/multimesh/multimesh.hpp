#pragma once

#include <wmtk/Mesh.hpp>
#include <memory>

namespace wmtk::components::multimesh {

enum class MultiMeshType { UV, Boundary, Tag };

std::pair<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> multimesh(
    const MultiMeshType& type,
    Mesh& parent,
    const std::shared_ptr<Mesh>& child,
    const attribute::MeshAttributeHandle parent_position_handle,
    const std::string& tag_name,
    const int64_t tag_value,
    const int64_t primitive);

} // namespace wmtk::components
