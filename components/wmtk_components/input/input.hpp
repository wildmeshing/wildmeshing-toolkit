#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>

namespace wmtk {
namespace components {
void input(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);
} // namespace components
} // namespace wmtk