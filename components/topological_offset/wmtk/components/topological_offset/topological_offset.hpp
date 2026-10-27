#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>


namespace wmtk::components::topological_offset {

void topological_offset(nlohmann::json json_params);
ComponentResult topological_offset(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::topological_offset
