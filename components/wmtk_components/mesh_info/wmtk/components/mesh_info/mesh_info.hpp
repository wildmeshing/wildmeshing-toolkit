#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk {
namespace components {

void mesh_info(const nlohmann::json& j, io::Cache& cache);

} // namespace components
} // namespace wmtk