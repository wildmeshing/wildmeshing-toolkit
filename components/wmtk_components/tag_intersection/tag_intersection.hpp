#pragma once
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>


#include <wmtk/io/Cache.hpp>

namespace wmtk::components {

void tag_intersection(const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
