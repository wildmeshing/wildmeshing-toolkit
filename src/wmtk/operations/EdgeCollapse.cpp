#include "EdgeCollapse.hpp"

#include <cassert>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include "utils/multi_mesh_edge_collapse.hpp"


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : MeshOperation(m)
{
    auto collect_attrs = [&](auto&& mesh) {
        // can have const variant values here so gotta filter htose out
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
            for (const auto& attr : mesh.custom_attributes()) {
                std::visit(
                    [&](auto&& tah) noexcept {
                        using T = typename std::decay_t<decltype(tah)>::Type;
                        m_new_attr_strategies.emplace_back(
                            std::make_shared<operations::CollapseNewAttributeStrategy<T>>(
                                attribute::MeshAttributeHandle(mesh, attr)));
                    },
                    attr);
            }
        }
    };

    multimesh::MultiMeshVisitor custom_attribute_collector(collect_attrs);
    custom_attribute_collector.execute_from_root(m);
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


std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>
EdgeCollapse::get_new_attribute_strategy(const attribute::MeshAttributeHandle& attribute) const
{
    assert(attribute.is_same_mesh(mesh()));

    for (auto& s : m_new_attr_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeCollapse::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>& other)
{
    assert(attribute.is_same_mesh(mesh()));

    for (size_t i = 0; i < m_new_attr_strategies.size(); ++i) {
        if (m_new_attr_strategies[i]->matches_attribute(attribute)) {
            m_new_attr_strategies[i] = other;
            m_new_attr_strategies[i]->update_handle_mesh(mesh()); // TODO: is this rihght?
            return;
        }
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeCollapse::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const wmtk::operations::CollapseBasicStrategy& strategy)
{
    std::visit(
        [&](auto&& val) -> void {
            using T = typename std::decay_t<decltype(val)>::Type;
            using OpType = operations::CollapseNewAttributeStrategy<T>;

            std::shared_ptr<OpType> tmp = std::make_shared<OpType>(attribute);
            tmp->set_strategy(strategy);

            set_new_attribute_strategy(attribute, tmp);
        },
        attribute.handle());
}

} // namespace wmtk::operations
