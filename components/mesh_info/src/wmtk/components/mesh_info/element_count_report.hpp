#pragma once
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk {
class Mesh;
namespace components::multimesh {
class MeshCollection;
}
} // namespace wmtk

namespace wmtk::components::mesh_info {

std::vector<int64_t> element_count_report(const Mesh& m);
nlohmann::json element_count_report_named(const Mesh& m);

nlohmann::json element_count_report_named(const components::multimesh::MeshCollection& m);
} // namespace wmtk::components::mesh_info
