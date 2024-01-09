#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/base/Paths.hpp>

namespace wmtk {
namespace components {

void mesh_info(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace components
} // namespace wmtk