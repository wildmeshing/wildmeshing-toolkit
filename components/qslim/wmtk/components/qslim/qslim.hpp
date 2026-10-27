#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

namespace wmtk::components::qslim {

void qslim(nlohmann::json json_params);
ComponentResult qslim(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::qslim