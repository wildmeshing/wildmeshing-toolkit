#include "CollapseEdgeOperation.hpp"
#include "SwapEdgeOperation.hpp"
#include "SwayEdgeOperation.hpp"
namespace wmtk {
SwapEdgeOperation::SwapEdgEOperation(TriMesh& m, Tuple& t)
    : TupleOperation(m, t)
{}


bool SwapEdgeOperation::execute()
{
    Tuple split_result;
    {
        SplitEdgeOperation split(m, m_input_tuple);
        if (!split()) {
            return false;
        }
        split_result = split.return_tuple().value();
    }
    // TODO: navigate collapse
    Tuple collapse_input = {};
    Tuple collapse_result;
    {
        CollapseEdgeOperation collapse(m, collapse_input);
        if (!collapse()) {
            return false;
        }
        collapse_result = collapse.return_tuple().value();
    }
}
} // namespace wmtk
