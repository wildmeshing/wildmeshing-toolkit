#pragma once
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>

namespace wmtk::components {

void regular_space(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);

} // namespace wmtk::components
