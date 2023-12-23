#pragma once
#include <map>

#include <nlohmann/json.hpp>

#include <filesystem>

namespace wmtk::components {

void wildmeshing(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);

} // namespace wmtk::components
