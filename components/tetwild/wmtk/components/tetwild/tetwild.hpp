#pragma once

#include <wmtk/MeshData.hpp>

#include <nlohmann/json.hpp>

#include "TetWildMesh.h"


namespace wmtk::components::tetwild {

TetWildMesh::ExportStruct tetwild_with_export(nlohmann::json json_params);

void tetwild(nlohmann::json json_params);
ComponentResult tetwild(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::tetwild
