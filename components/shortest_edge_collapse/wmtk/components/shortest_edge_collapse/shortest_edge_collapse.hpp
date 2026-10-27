#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

namespace wmtk::components::shortest_edge_collapse {

void shortest_edge_collapse(nlohmann::json json_params);
ComponentResult shortest_edge_collapse(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::shortest_edge_collapse