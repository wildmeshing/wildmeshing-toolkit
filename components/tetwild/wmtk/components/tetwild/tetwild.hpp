#pragma once

#include <nlohmann/json.hpp>

#include "TetWildMesh.h"


namespace wmtk::components::tetwild {

TetWildMesh::ExportStruct tetwild_with_export(nlohmann::json json_params);

void tetwild(nlohmann::json json_params);

} // namespace wmtk::components::tetwild
