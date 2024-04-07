#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/base/Paths.hpp>


namespace wmtk::components {

/**
 * @brief Perform maching tetrahedra/triangles.
 */
void marching_window(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
