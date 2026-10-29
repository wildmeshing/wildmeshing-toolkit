#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

namespace wmtk::components::surface_map {

void surface_map(nlohmann::json json_params);
/// The memory route: operation info only, its report returned. `history` is still a file, and
/// map and transfer, whose points and meshes are files, throw.
ComponentResult surface_map(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::surface_map
