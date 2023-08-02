#include "TriMeshSwapEdgeOperation.hpp"
#include "TriMeshSplitEdgeOperation.hpp"
#include "TriMeshCollapseEdgeOperation.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
namespace wmtk {
TriMeshSwapEdgeOperation::TriMeshSwapEdgeOperation(Mesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{}

std::string TriMeshSwapEdgeOperation::name() const
{
    return "TriMeshSwapEdgeOperation";
}

bool TriMeshSwapEdgeOperation::before() const
{   
    if (!m_mesh.is_valid(m_input_tuple))
    {
        return false;
    }
    // not allowing boundary edges to be swapped
    return (!m_mesh.is_boundary(m_input_tuple));
}

Tuple TriMeshSwapEdgeOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshSwapEdgeOperation::execute()
{
    Tuple split_result;
    TriMeshSplitEdgeOperation split(m_mesh, m_input_tuple);
    if (!split()) {
        return false;
    }
    split_result = split.return_tuple();

    // TODO: navigate collapse
    Tuple collapse_input = m_mesh.switch_tuple(m_mesh.switch_tuple(split_result, PrimitiveType::Vertex),PrimitiveType::Edge);
    TriMeshCollapseEdgeOperation collapse(m_mesh, collapse_input);
    if (!collapse()) {
        return false;
    }
    m_output_tuple = collapse.return_tuple();

    return true;
}


} // namespace wmtk
