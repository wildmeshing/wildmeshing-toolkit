#include "multi_mesh_edge_collapse.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshSimplexEventVisitor.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeCollapseFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<invariants::InvariantCollection> multimesh_edge_collapse_invariants(const Mesh& m)
{
    auto invariants = std::make_shared<invariants::InvariantCollection>(m);
    //*invariants = basic_multimesh_invariant_collection(m, PrimitiveType::Edge);
    return invariants;
}

CollapseReturnData multi_mesh_edge_collapse(
    Mesh& mesh,
    const Tuple& t,
    const std::vector<std::shared_ptr<const operations::BaseCollapseNewAttributeStrategy>>&
        new_attr_strategies)
{
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs over edges
        MultiMeshEdgeCollapseFunctor{});
    visitor.execute_from_root(mesh, simplex::Simplex(mesh, PrimitiveType::Edge, t));

    multimesh::MultiMeshSimplexEventVisitor event_visitor(visitor);
    event_visitor.run_on_nodes(UpdateEdgeOperationMultiMeshMapFunctor{});

    auto cache = visitor.take_cache();

    auto tuples = wmtk::multimesh::operations::extract_operation_tuples(cache);
    auto update_attributes = [&](auto&& m) {
        using T = std::remove_reference_t<decltype(m)>;
        if constexpr (!std::is_const_v<T>) {
            for (const auto& collapse_ptr : new_attr_strategies) {
                if (&m == &collapse_ptr->mesh()) {
                    collapse_ptr->update(m, cache, tuples);
                }
            }
        }
    };

    multimesh::MultiMeshVisitor(update_attributes).execute_from_root(mesh);

    return cache;
}
std::vector<simplex::Simplex> multi_mesh_edge_collapse_with_modified_simplices(
    Mesh& mesh,
    const simplex::Simplex& simplex,
    const std::vector<std::shared_ptr<const operations::BaseCollapseNewAttributeStrategy>>&
        new_attr_strategies)
{
    int64_t simplex_id = mesh.id(simplex);
    auto return_data =
        operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple(), new_attr_strategies);

    if (mesh.is_free()) {
        return std::vector<simplex::Simplex>{1};
    }
    const auto& var = return_data.get_variant(mesh, simplex_id);

    // const auto& var =
    //     mesh.parent_scope([&]() -> const auto& { return return_data.get_variant(mesh, simplex);
    //     });

    return std::visit(
        [&mesh](const auto& rt) -> std::vector<simplex::Simplex> {
            return {simplex::Simplex::vertex(mesh, rt.m_output_tuple)};
        },
        var);
}
} // namespace wmtk::operations::utils
