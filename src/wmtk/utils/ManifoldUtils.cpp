#include "ManifoldUtils.hpp"

#include <wmtk/utils/VectorUtils.h>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <lagrange/mesh_cleanup/resolve_nonmanifoldness.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/extract_manifold_patches.h>
#include <igl/is_edge_manifold.h>
#include <igl/remove_unreferenced.h>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <Eigen/Core>

namespace wmtk::manifold_internal {

void resolve_nonmanifoldness(Vertices& V, Facets& F, std::vector<size_t>& modified_vertices) {
    auto mesh = lagrange::create_mesh(std::move(V), std::move(F));
    mesh = lagrange::resolve_nonmanifoldness(*mesh);
    mesh->export_vertices(V);
    mesh->export_facets(F);
}

} // namespace wmtk::manifold_internal

bool wmtk::separate_to_manifold(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Eigen::Vector3d>& out_v,
    std::vector<std::array<size_t, 3>>& out_f,
    std::vector<size_t>& mod_v)
{
    using Vertices = wmtk::manifold_internal::Vertices;
    using Facets = wmtk::manifold_internal::Facets;

    Vertices V(vertices.size(), 3);
    for (auto i = 0; i < vertices.size(); i++) V.row(i) = vertices[i];
    Facets F(faces.size(), 3);
    for (auto i = 0; i < faces.size(); i++) F.row(i) << faces[i][0], faces[i][1], faces[i][2];

    wmtk::manifold_internal::resolve_nonmanifoldness(V, F, mod_v);

    out_v.resize(V.rows());
    out_f.resize(F.rows());
    for (auto i = 0; i < V.rows(); i++) out_v[i] = V.row(i);
    for (auto i = 0; i < F.rows(); i++)
        out_f[i] =
            std::array<size_t, 3>{{(size_t)faces[i][0], (size_t)faces[i][1], (size_t)faces[i][2]}};

    return true;
}
