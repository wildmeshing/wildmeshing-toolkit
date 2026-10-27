#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

namespace wmtk::components::isotropic_remeshing {

void isotropic_remeshing(nlohmann::json json_params);
ComponentResult isotropic_remeshing(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::isotropic_remeshing