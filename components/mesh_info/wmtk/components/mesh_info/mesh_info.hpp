#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/utils/Paths.hpp>

namespace wmtk {
namespace components {

void mesh_info(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace components
} // namespace wmtk