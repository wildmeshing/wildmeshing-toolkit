#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/io/Cache.hpp>

#include <nlohmann/json.hpp>

namespace wmtk::components::utils {

attribute::MeshAttributeHandle get_attribute(const Mesh& m, const std::string& name);

attribute::MeshAttributeHandle
get_attribute(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute);

std::vector<attribute::MeshAttributeHandle>
get_attributes(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute_names);

} // namespace wmtk::components::utils