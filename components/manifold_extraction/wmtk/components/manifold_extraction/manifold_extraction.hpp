#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>


namespace wmtk::components::manifold_extraction {

void manifold_extraction(nlohmann::json json_params);
ComponentResult manifold_extraction(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::manifold_extraction
