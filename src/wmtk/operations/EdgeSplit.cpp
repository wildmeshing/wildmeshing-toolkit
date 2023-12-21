#include "EdgeSplit.hpp"

namespace wmtk::operations {

EdgeSplit::EdgeSplit(Mesh& m)
    : MeshOperation(m)
{}

///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(EdgeMesh& mesh, const Simplex& simplex)
{
    throw std::runtime_error("split not implemented for edge mesh");
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const EdgeMesh& mesh, onst Simplex& simplex)
{
    throw std::runtime_error("split not implemented for edge mesh");
}
///////////////////////////////


///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(TriMesh& mesh, const Simplex& simplex)
{
    auto return_data = operations::utils::multi_mesh_edge_split(mesh(), simplex.tuple());

    spdlog::trace("{}", primitive_type_name(simplex.primitive_type()));

    const operations::tri_mesh::EdgeOperationData& my_data = return_data.get(tri_mesh(), simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const TriMesh& mesh, onst Simplex& simplex)
    const
{
    return {simplex};
}
///////////////////////////////


///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(TetMesh& mesh, const Simplex& simplex)
{
    auto return_data = mesh.split_edge(input_tuple(), hash_accessor());

    return {simplex::Simplex::vertex(return_data.m_output_tuple)};
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const TetMesh& mesh, onst Simplex& simplex)
{
    return {simplex};
}
///////////////////////////////


} // namespace wmtk::operations
