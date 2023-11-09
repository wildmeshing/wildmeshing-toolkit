#pragma once

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);
} // namespace components
} // namespace wmtk