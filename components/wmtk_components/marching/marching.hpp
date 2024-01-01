#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk::components {

/**
 * @brief Perform maching tetrahedra/triangles.
 */
void marching(const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
