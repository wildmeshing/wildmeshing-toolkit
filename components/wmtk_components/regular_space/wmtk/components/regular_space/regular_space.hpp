#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/base/Paths.hpp>

namespace wmtk::components {

void regular_space(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
