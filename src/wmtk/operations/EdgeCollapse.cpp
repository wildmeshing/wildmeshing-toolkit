#include "EdgeCollapse.hpp"

#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include "tri_mesh/BasicCollapseNewAttributeStrategy.hpp"
#include "tri_mesh/PredicateAwareCollapseNewAttributeStrategy.hpp"

#include "utils/multi_mesh_edge_collapse.hpp"


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : MeshOperation(m)
{
    // PredicateAwareCollapseNewAttributeStrategy BasicCollapseNewAttributeStrategy

    const int top_cell_dimension = m.top_cell_dimension();

    for (auto& attr : m.m_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;

                if (top_cell_dimension == 2)
                    m_new_attr_strategies.emplace_back(
                        std::make_shared<
                            operations::tri_mesh::BasicCollapseNewAttributeStrategy<T>>(val));
                else {
                    throw std::runtime_error("collapse not implemented for edge/tet mesh");
                }
            },
            attr);

        m_new_attr_strategies.back()->update_handle_mesh(m);
    }
}

////////////////////////////////////
std::vector<Simplex> EdgeCollapse::execute(EdgeMesh& mesh, const Simplex& simplex)
{
    throw std::runtime_error("collapse not implemented for edge mesh");
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives(
    const EdgeMesh& mesh,
    const Simplex& simplex) const
{
    throw std::runtime_error("collapse not implemented for edge mesh");
}
////////////////////////////////////


////////////////////////////////////
std::vector<Simplex> EdgeCollapse::execute(TriMesh& mesh, const Simplex& simplex)
{
    auto return_data =
        operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple(), m_new_attr_strategies);

    const operations::tri_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {Simplex(PrimitiveType::Vertex, my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives(
    const TriMesh& mesh,
    const Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh.parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())); });
    return {v0, v1};
}
////////////////////////////////////


////////////////////////////////////
std::vector<Simplex> EdgeCollapse::execute(TetMesh& mesh, const Simplex& simplex)
{
    auto return_data = operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple());
    const operations::tet_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);
    return {Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives(
    const TetMesh& mesh,
    const Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh.parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())); });
    return {v0, v1};
}
////////////////////////////////////


void EdgeCollapse::set_standard_strategy(
    const attribute::MeshAttributeHandleVariant& attribute,
    const wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy& strategy)
{
    std::visit(
        [&](auto&& val) -> void {
            using T = typename std::decay_t<decltype(val)>::Type;
            using PACNAS = operations::tri_mesh::PredicateAwareCollapseNewAttributeStrategy<T>;

            std::shared_ptr<PACNAS> tmp = std::make_shared<PACNAS>(val, mesh());
            tmp->set_standard_collapse_strategy(strategy);

            set_strategy(attribute, tmp);
        },
        attribute);
}

} // namespace wmtk::operations
