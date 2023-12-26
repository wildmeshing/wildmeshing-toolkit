#include "EdgeCollapse.hpp"

#include "tri_mesh/PredicateAwareCollapseNewAttributeStrategy.hpp"

#include "utils/multi_mesh_edge_collapse.hpp"


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : MeshOperation(m)
{
    const int top_cell_dimension = m.top_cell_dimension();

    for (auto& attr : m.m_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = std::decay_t<decltype(val)>::Type;

                if (top_cell_dimension == 2)
                    m_new_attr_strategies.emplace_back(
                        std::make_shared<
                            operations::tri_mesh::PredicateAwareCollapseNewAttributeStrategy<T>>(
                            val));
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
    auto return_data = operations::utils::multi_mesh_edge_collapse(mesh, simplex.tuple());

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
    Accessor<long> accessor = hash_accessor();
    auto return_data = mesh.collapse_edge(simplex.tuple(), accessor);
    return {Simplex::vertex(return_data.m_output_tuple)};
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

} // namespace wmtk::operations
