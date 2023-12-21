#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>

#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>

namespace wmtk::operations::tri_mesh {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : TriMeshOperation(m)
{}

std::vector<Simplex> EdgeCollapse::execute(const Simplex& simplex)
{
    auto return_data = operations::utils::multi_mesh_edge_collapse(mesh(), simplex.tuple());

    const operations::tri_mesh::EdgeOperationData& my_data = return_data.get(tri_mesh(), simplex);

    return {Simplex(PrimitiveType::Vertex, my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives(const Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh().parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh().switch_vertex(simplex.tuple())); });
    return {v0, v1};
}

} // namespace wmtk::operations::tri_mesh
