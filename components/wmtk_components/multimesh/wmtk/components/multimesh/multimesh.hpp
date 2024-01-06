#pragma once
#include <map>

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk::components {

void multimesh(const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
