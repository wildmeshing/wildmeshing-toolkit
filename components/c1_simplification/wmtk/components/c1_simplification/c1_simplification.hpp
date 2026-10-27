#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>

namespace wmtk::components::c1_simplification {

void c1_simplification(nlohmann::json json_params);
ComponentResult c1_simplification(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::c1_simplification
