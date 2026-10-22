#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>

#include "TriWildMesh.h"


namespace wmtk::components::triwild {

void triwild(nlohmann::json json_params);
ComponentResult triwild(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::triwild
