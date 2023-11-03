#include "ManifoldUtils.hpp"
#include <wmtk/utils/VectorUtils.h>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <lagrange/mesh_cleanup/resolve_nonmanifoldness.h>
#include <igl/extract_manifold_patches.h>
#include <igl/is_edge_manifold.h>
#include <igl/remove_unreferenced.h>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <vector>

namespace wmtk::manifold_internal {

void resolve_nonmanifoldness(Vertices& V, Facets& F, std::vector<size_t>& modified_vertices)
{
    using Scalar = Vertices::Scalar;
    using Index = Facets::Scalar;
    using Mesh = lagrange::Mesh<Vertices, Facets>;
    using AttributeArray = Mesh::AttributeArray;

    constexpr Index Invalid = std::numeric_limits<Index>::max();

    // For this, we assume that sizeof(double) == sizeof(uint64_t), which should be the case on all
    // platforms...
    static_assert(
        sizeof(Scalar) == sizeof(Index),
        "Vertex scalar type and facet index type should have the same size");
    auto mesh = lagrange::create_mesh(std::move(V), std::move(F));

    // Create a vertex attribute to track vertex id
    AttributeArray old_vids(mesh->get_num_vertices(), 1);
    for (Index v = 0; v < mesh->get_num_vertices(); ++v) {
        old_vids(v, 0) = static_cast<double>(v);
    }
    mesh->add_vertex_attribute("old_vids");
    mesh->import_vertex_attribute("old_vids", std::move(old_vids));

    const Index old_num_vertices = mesh->get_num_vertices();

    // Resolve non-manifoldness
    mesh = lagrange::resolve_nonmanifoldness(*mesh);
    const Index new_num_vertices = mesh->get_num_vertices();

    // Export vertices + facets + original ids
    mesh->export_vertices(V);
    mesh->export_facets(F);
    mesh->export_vertex_attribute("old_vids", old_vids);

    // Check which vids has been duplicated
    std::vector<Index> old_to_new(old_num_vertices, Invalid);
    std::vector<bool> is_vertex_modified(new_num_vertices, false);
    for (Index new_idx = 0; new_idx < new_num_vertices; ++new_idx) {
        Index old_idx = static_cast<double>(old_vids(new_idx, 0));
        assert(old_idx < old_num_vertices);
        if (old_to_new[old_idx] != Invalid) {
            is_vertex_modified[old_to_new[old_idx]] = true;
            is_vertex_modified[new_idx] = true;
        } else {
            old_to_new[old_idx] = new_idx;
        }
    }

    // List all duplicated vertices
    modified_vertices.clear();
    for (size_t v = 0; v < is_vertex_modified.size(); ++v) {
        if (is_vertex_modified[v]) {
            modified_vertices.push_back(v);
        }
    }
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
        out_f[i] = std::array<size_t, 3>{{(size_t)F(i, 0), (size_t)F(i, 1), (size_t)F(i, 2)}};

    return true;
}
