#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk::components {

void output(const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components