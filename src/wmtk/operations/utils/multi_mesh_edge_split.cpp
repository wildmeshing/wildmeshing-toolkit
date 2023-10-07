#include "multi_mesh_edge_split.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<InvariantCollection> multimesh_edge_split_invariants(const Mesh& m)
{
    return std::make_shared<InvariantCollection>();
}

void multi_mesh_edge_split(Mesh& mesh, const Tuple& t)
{
    multimesh::MultiMeshVisitor visitor(
        MultiMeshEdgeSplitFunctor{},
        UpdateEdgeOperationMultiMeshMapFunctor{});
    visitor.execute_from_root(mesh, Simplex(PrimitiveType::Edge, t));
}
} // namespace wmtk::operations::utils
