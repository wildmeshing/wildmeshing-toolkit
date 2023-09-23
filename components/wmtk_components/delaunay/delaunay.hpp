#pragma once
#include <map>
#include <nlohmann/json.hpp>

namespace wmtk::components {

void delaunay(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files);

} // namespace wmtk::components
