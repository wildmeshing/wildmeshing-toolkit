

#include "CollapseEdgeOperation.hpp"
namespace wmtk {
CollapseEdgeOperation::SplitEdgeOperation(TriMesh& m, Tuple& t)
    : TupleOperation(m, t)
{}


bool CollapseEdgeOperation::execute()
{
    return m_mesh.collapse_edge(m_input_tuple);
}
} // namespace wmtk
