#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/utils/Paths.hpp>

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::components {

void fusion(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components