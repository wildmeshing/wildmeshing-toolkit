#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/utils/Paths.hpp>

namespace wmtk::components {

void shortestedge_collapse(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components