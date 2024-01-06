#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk {
namespace components {

void get_all_meshes(const nlohmann::json& j, io::Cache& cache);

} // namespace components
} // namespace wmtk