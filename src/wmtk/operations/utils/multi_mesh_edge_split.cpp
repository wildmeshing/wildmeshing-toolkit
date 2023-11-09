#include "multi_mesh_edge_split.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
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
    multimesh::MultiMeshVisitor visitor(
        std::integral_constant<long, 1>{}, // specify that this runs on edges
        MultiMeshEdgeSplitFunctor{},
        UpdateEdgeOperationMultiMeshMapFunctor{});
    return visitor.execute_from_root(mesh, Simplex(PrimitiveType::Edge, t));
}
} // namespace wmtk::operations::utils
