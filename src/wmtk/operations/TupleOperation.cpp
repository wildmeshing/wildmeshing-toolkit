#include "TupleOperation.hpp"


namespace wmtk {
TupleOperation::TupleOperation(Mesh& m, const Tuple& t)
    : Operation(m)
    , m_input_tuple(t)
{}
} // namespace wmtk
