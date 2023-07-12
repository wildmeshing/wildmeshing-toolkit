
#include "SplitEdgeOperation.hpp"
namespace wmtk {
SplitEdgeOperation::SplitEdgeOperation(TriMesh& m, Tuple& t)
    : TupleOperation(m, t)
{}


bool SplitEdgeOperation::execute()
{
    return m_mesh.split_edge(m_input_tuple);
}
} // namespace wmtk
