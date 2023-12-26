#include "EdgeCollapse.hpp"

#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : MeshOperation(m)
{}

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


} // namespace wmtk::operations
