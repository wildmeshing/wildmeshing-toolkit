#pragma once
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk::components {

/**
 * @brief Generate a multi-mesh from a mesh with a tag that represents the substructure
 */
void multimesh_from_tag(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
