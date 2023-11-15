#pragma once
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>

namespace wmtk::components {

void embedded_remeshing(std::map<std::string, std::filesystem::path>& files);

} // namespace wmtk::components
