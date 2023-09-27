#pragma once

#include <igl/readOFF.h>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>
#include "internal/MeshInfoOptions.hpp"

namespace wmtk {
namespace components {
void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);
} // namespace components
} // namespace wmtk