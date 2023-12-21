

#include "EdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_split.hpp>

namespace wmtk::operations::tri_mesh {

EdgeSplit::EdgeSplit(Mesh& m)
    : TriMeshOperation(m)
{}

std::vector<Simplex> EdgeSplit::execute(const Simplex& simplex)
{
    auto return_data = operations::utils::multi_mesh_edge_split(mesh(), simplex.tuple());

    spdlog::trace("{}", primitive_type_name(simplex.primitive_type()));

    const operations::tri_mesh::EdgeOperationData& my_data = return_data.get(tri_mesh(), simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::tri_mesh
