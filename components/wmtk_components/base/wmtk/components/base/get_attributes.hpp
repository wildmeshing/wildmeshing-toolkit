#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/io/Cache.hpp>

#include <nlohmann/json.hpp>

namespace wmtk::components::base {

std::vector<attribute::MeshAttributeHandle>
get_attributes(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute_names);

}