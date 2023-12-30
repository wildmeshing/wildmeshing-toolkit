#include "multi_mesh_edge_collapse.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshSimplexEventVisitor.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeCollapseFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<InvariantCollection> multimesh_edge_collapse_invariants(const Mesh& m)
{
    auto invariants = std::make_shared<InvariantCollection>(m);
    //*invariants = basic_multimesh_invariant_collection(m, PrimitiveType::Edge);
    return invariants;
}

CollapseReturnData multi_mesh_edge_collapse(
    Mesh& mesh,
    const Tuple& t,
    const std::vector<std::shared_ptr<operations::NewAttributeStrategy>>& new_attr_strategies)
{
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs over edges
        MultiMeshEdgeCollapseFunctor{});
    visitor.execute_from_root(mesh, simplex::Simplex(PrimitiveType::Edge, t));

    multimesh::MultiMeshSimplexEventVisitor event_visitor(visitor);
    event_visitor.run_on_nodes(UpdateEdgeOperationMultiMeshMapFunctor{});

    auto cache = visitor.cache();

    auto tuples = wmtk::multimesh::operations::extract_operation_tuples(cache);
    auto update_attributes = [&](auto&& m) {
        using T = std::remove_reference_t<decltype(m)>;
        if constexpr (!std::is_const_v<T>) {
            for (const auto& collapse_ptr : new_attr_strategies) {
                static_cast<CollapseNewAttributeStrategy*>(collapse_ptr.get())
                    ->update(cache, tuples);
            }
        }
    };

    multimesh::MultiMeshVisitor(update_attributes).execute_from_root(mesh);

    return cache;
}
} // namespace wmtk::operations::utils
