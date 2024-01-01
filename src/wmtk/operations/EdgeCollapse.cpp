#include "EdgeCollapse.hpp"

#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

#include "utils/multi_mesh_edge_collapse.hpp"


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : MeshOperation(m)
{
    const int top_cell_dimension = m.top_cell_dimension();

    for (const auto& attr : m.custom_attributes()) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;
                m_new_attr_strategies.emplace_back(
                    std::make_shared<operations::CollapseNewAttributeStrategy<T>>(
                        attribute::MeshAttributeHandle<T>(m, val)));
            },
            attr);

        m_new_attr_strategies.back()->update_handle_mesh(m);
    }
}

////////////////////////////////////
std::vector<simplex::Simplex> EdgeCollapse::execute_aux(
    EdgeMesh& mesh,
    const simplex::Simplex& simplex)
{
    auto return_data =
        operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple(), m_new_attr_strategies);

    const operations::edge_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex(PrimitiveType::Vertex, my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives_aux(
    const EdgeMesh& mesh,
    const simplex::Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh.parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())); });
    return {v0, v1};
}
////////////////////////////////////


////////////////////////////////////
std::vector<simplex::Simplex> EdgeCollapse::execute_aux(
    TriMesh& mesh,
    const simplex::Simplex& simplex)
{
    auto return_data =
        operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple(), m_new_attr_strategies);

    const operations::tri_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex(PrimitiveType::Vertex, my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives_aux(
    const TriMesh& mesh,
    const simplex::Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh.parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())); });
    return {v0, v1};
}
////////////////////////////////////


////////////////////////////////////
std::vector<simplex::Simplex> EdgeCollapse::execute_aux(
    TetMesh& mesh,
    const simplex::Simplex& simplex)
{
    auto return_data =
        operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple(), m_new_attr_strategies);
    const operations::tet_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);
    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives_aux(
    const TetMesh& mesh,
    const simplex::Simplex& simplex) const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
    const simplex::Simplex v1 = mesh.parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh.switch_vertex(simplex.tuple())); });
    return {v0, v1};
}
////////////////////////////////////


void EdgeCollapse::set_new_attribute_strategy(
    const attribute::MeshAttributeHandleVariant& attribute,
    const wmtk::operations::CollapseBasicStrategy& strategy)
{
    std::visit(
        [&](auto&& val) -> void {
            using T = typename std::decay_t<decltype(val)>::Type;
            using OpType = operations::CollapseNewAttributeStrategy<T>;

            std::shared_ptr<OpType> tmp = std::make_shared<OpType>(val);
            tmp->set_collapse_strategy(strategy);

            Operation::set_new_attribute_strategy(attribute, tmp);
        },
        attribute);
}

} // namespace wmtk::operations
