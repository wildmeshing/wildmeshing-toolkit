#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>


namespace wmtk::components::simwild {

void simwild(nlohmann::json json_params);
ComponentResult simwild(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::simwild
