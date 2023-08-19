
#include "TupleOperation.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk {
TupleOperation::TupleOperation(Mesh& m, const InvariantCollection& invariants, const Tuple& t)
    : Operation(m)
    , m_invariants(invariants)
    , m_input_tuple{t}
{}
const Tuple& TupleOperation::input_tuple() const
{
    return m_input_tuple;
}

bool TupleOperation::before() const
{
    return m_invariants.before(input_tuple());
}
bool TupleOperation::after() const
{
    return m_invariants.after(PrimitiveType::Face, modified_primitives(PrimitiveType::Face));
}
std::vector<Tuple> TupleOperation::modified_primitives(PrimitiveType) const
{
    return {};
}
} // namespace wmtk

