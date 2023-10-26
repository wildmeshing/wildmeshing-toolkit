#include "multi_mesh_edge_collapse.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeCollapseFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<InvariantCollection> multimesh_edge_collapse_invariants(const Mesh& m)
{
    auto invariants = std::make_shared<InvariantCollection>();
    *invariants = basic_multimesh_invariant_collection(m, PrimitiveType::Edge);
    return invariants;
}

CollapseReturnData multi_mesh_edge_collapse(Mesh& mesh, const Tuple& t)
{
    multimesh::MultiMeshVisitor visitor(
        std::integral_constant<long, 1>{}, // specify that this runs over edges
        MultiMeshEdgeCollapseFunctor{},
        UpdateEdgeOperationMultiMeshMapFunctor{});
    return visitor.execute_from_root(mesh, Simplex(PrimitiveType::Edge, t));
}
} // namespace wmtk::operations::utils
