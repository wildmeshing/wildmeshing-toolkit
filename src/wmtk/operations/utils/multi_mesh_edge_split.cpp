#include "multi_mesh_edge_split.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshSimplexEventVisitor.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<InvariantCollection> multimesh_edge_split_invariants(const Mesh& m)
{
    auto invariants = std::make_shared<InvariantCollection>();
    *invariants = basic_multimesh_invariant_collection(m, PrimitiveType::Edge);
    return invariants;
}

SplitReturnData multi_mesh_edge_split(Mesh& mesh, const Tuple& t)
{
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<long, 1>{}, // specify that this runs on edges
        MultiMeshEdgeSplitFunctor{});
    visitor.execute_from_root(mesh, Simplex(PrimitiveType::Edge, t));
    multimesh::MultiMeshSimplexEventVisitor event_visitor(visitor);
    event_visitor.run_on_edges(UpdateEdgeOperationMultiMeshMapFunctor{});
    event_visitor.run_on_nodes(UpdateEdgeOperationMultiMeshMapFunctor{});

    return visitor.cache();
}
} // namespace wmtk::operations::utils
