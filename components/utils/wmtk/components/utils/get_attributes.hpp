#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/io/Cache.hpp>

#include <nlohmann/json.hpp>

namespace wmtk::components::utils {

/**
 * @brief Get an attribute handle only by name.
 *
 * If there is no or multiple attributes, an error is thrown.
 *
 * @param m Mesh holding the attribute.
 * @param name Name of the attribute.
 */
attribute::MeshAttributeHandle get_attribute(const Mesh& m, const std::string& name);

attribute::MeshAttributeHandle
get_attribute(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute);

std::vector<attribute::MeshAttributeHandle>
get_attributes(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute_names);

} // namespace wmtk::components::utils