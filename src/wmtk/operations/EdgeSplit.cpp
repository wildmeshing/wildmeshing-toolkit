#include "EdgeSplit.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "utils/multi_mesh_edge_split.hpp"

namespace wmtk::operations {

EdgeSplit::EdgeSplit(Mesh& m)
    : MeshOperation(m)
{}

///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(EdgeMesh& mesh, const Simplex& simplex)
{
    throw std::runtime_error("Split not implemented for edge mesh");
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const EdgeMesh& mesh, const Simplex& simplex)
    const
{
    throw std::runtime_error("Split not implemented for edge mesh");
}
///////////////////////////////


///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(TriMesh& mesh, const Simplex& simplex)
{
    auto return_data = utils::multi_mesh_edge_split(mesh, simplex.tuple());

    spdlog::trace("{}", primitive_type_name(simplex.primitive_type()));

    const tri_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const TriMesh& mesh, const Simplex& simplex)
    const
{
    return {simplex};
}
///////////////////////////////


///////////////////////////////
std::vector<Simplex> EdgeSplit::execute(TetMesh& mesh, const Simplex& simplex)
{
    Accessor<long> accessor = hash_accessor();
    auto return_data = mesh.split_edge(simplex.tuple(), accessor);

    return {simplex::Simplex::vertex(return_data.m_output_tuple)};
}

std::vector<Simplex> EdgeSplit::unmodified_primitives(const TetMesh& mesh, const Simplex& simplex)
    const
{
    return {simplex};
}
///////////////////////////////

std::pair<Tuple, Tuple> EdgeSplit::new_spine_edges(const Mesh& mesh, const Tuple& new_vertex)
{
    // new_vertex is a spine edge on a face pointing to the new vertex, so we
    // * PE -> new edge
    // * PF -> other face
    // * PE -> other spine edge
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    return {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PE})};
}

} // namespace wmtk::operations
