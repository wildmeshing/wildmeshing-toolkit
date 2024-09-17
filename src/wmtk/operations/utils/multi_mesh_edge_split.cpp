#include "multi_mesh_edge_split.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/multimesh/MultiMeshSimplexEventVisitor.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/operations/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::utils {

std::shared_ptr<invariants::InvariantCollection> multimesh_edge_split_invariants(const Mesh& m)
{
    auto invariants = std::make_shared<invariants::InvariantCollection>(m);
    //*invariants = basic_multimesh_invariant_collection(m, PrimitiveType::Edge);
    return invariants;
}

SplitReturnData multi_mesh_edge_split(
    Mesh& mesh,
    const Tuple& t,
    const std::vector<std::shared_ptr<const operations::BaseSplitNewAttributeStrategy>>&
        new_attr_strategies)
{
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs on edges
        MultiMeshEdgeSplitFunctor{});
    visitor.execute_from_root(mesh, simplex::Simplex(mesh, PrimitiveType::Edge, t));
    multimesh::MultiMeshSimplexEventVisitor event_visitor(visitor);
    event_visitor.run_on_edges(UpdateEdgeOperationMultiMeshMapFunctor{});
    event_visitor.run_on_nodes(UpdateEdgeOperationMultiMeshMapFunctor{});


    auto cache = visitor.take_cache();

    auto tuples = wmtk::multimesh::operations::extract_operation_tuples(cache);
    auto update_attributes = [&](auto&& m) {
        using T = std::remove_reference_t<decltype(m)>;
        if constexpr (!std::is_const_v<T>) {
            for (const auto& split_ptr : new_attr_strategies) {
                if (&m == &split_ptr->mesh()) {
                    split_ptr->update(m, cache, tuples);
                }
            }
        }
    };

    multimesh::MultiMeshVisitor(update_attributes).execute_from_root(mesh);

    return cache;
}

std::vector<simplex::Simplex> multi_mesh_edge_split_with_modified_simplices(
    Mesh& mesh,
    const simplex::Simplex& simplex,
    const std::vector<std::shared_ptr<const operations::BaseSplitNewAttributeStrategy>>&
        new_attr_strategies)
{
    auto candidates = top_dimension_cofaces(mesh, simplex);
    auto return_data = multi_mesh_edge_split(mesh, simplex.tuple(), new_attr_strategies);

    for (const auto& c : candidates) {
        if (return_data.has_variant(mesh, simplex::Simplex::edge(mesh, c.tuple()))) {
            return std::visit(
                [&mesh](const auto& rt) -> std::vector<simplex::Simplex> {
                    if (mesh.is_free()) {
                        return rt.new_vertices(mesh);
                    } else {
                        return {simplex::Simplex::vertex(mesh, rt.m_output_tuple)};
                    }
                },
                return_data.get_variant(mesh, simplex::Simplex::edge(mesh, c.tuple())));
        }
    }

    assert(return_data.has_variant(mesh, simplex));

    return {};


    // return std::visit(
    //     [&mesh](const auto& rt) -> std::vector<simplex::Simplex> {
    //         if (mesh.is_free()) {
    //             return rt.new_vertices(mesh);
    //         } else {
    //             return {simplex::Simplex::vertex(mesh, rt.m_output_tuple)};
    //         }
    //     },
    //     return_data.get_variant(mesh, simplex));
}
} // namespace wmtk::operations::utils
