#include "EdgeSplit.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>

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
    auto return_data = utils::multi_mesh_edge_split(mesh, simplex.tuple());

    spdlog::trace("{}", primitive_type_name(simplex.primitive_type()));

    const tet_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    new_edges = my_data.new_simplex_tuples(mesh, PrimitiveType::Edge);
    new_faces = my_data.new_simplex_tuples(mesh, PrimitiveType::Face);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
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
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::pair<Tuple, Tuple> ret;

    switch (mesh.top_simplex_type()) {
    case PE: ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE})};
    case PF: ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PE})};
    case PT: ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PT, PF, PE})};
    }
    return ret;
}

std::vector<Tuple> EdgeSplit::new_edge_tuples()
{
    return new_edges;
}

std::vector<Tuple> EdgeSplit::new_face_tuples()
{
    return new_faces;
}

} // namespace wmtk::operations
