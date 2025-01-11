#include "element_count_report.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/utils/primitive_range.hpp>


namespace wmtk::applications::utils {

std::vector<int64_t> element_count_report(const Mesh& m)
{
    std::vector<int64_t> ret;
    for (PrimitiveType pt :
         wmtk::utils::primitive_below(m.top_simplex_type(), /*lower_to_upper=*/true)) {
        ret.emplace_back(m.get_all(pt).size());
    }
    return ret;
}
nlohmann::json element_count_report_named(const Mesh& m)
{
    const static std::array<std::string_view, 4> names{"vertices", "edges", "faces", "cells"};

    const auto sizes = element_count_report(m);

    assert(sizes.size() <= names.size());
    nlohmann::json js;
    for (size_t i = 0; i < sizes.size(); ++i) {
        js[names[i]] = sizes[i];
    }
    return js;
}

nlohmann::json element_count_report_named(const components::multimesh::MeshCollection& meshes)
{
    nlohmann::json out;
    for (const auto& [name, mesh] : meshes.all_meshes()) {
        out[name] = wmtk::applications::utils::element_count_report_named(mesh);
    }
    return out;
}
} // namespace wmtk::applications::utils
