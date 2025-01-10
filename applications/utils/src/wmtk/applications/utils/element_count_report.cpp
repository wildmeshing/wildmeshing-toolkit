#include "element_count_report.hpp"

#include <wmtk/components/mesh_info/element_count_report.hpp>

namespace wmtk::applications::utils {

std::vector<int64_t> element_count_report(const Mesh& m)
{
    return wmtk::components::mesh_info::element_count_report(m);
}
nlohmann::json element_count_report_named(const Mesh& m)
{
    return wmtk::components::mesh_info::element_count_report_named(m);
}

nlohmann::json element_count_report_named(const components::multimesh::MeshCollection& m)
{
    return wmtk::components::mesh_info::element_count_report_named(m);
}
} // namespace wmtk::applications::utils
