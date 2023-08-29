
#include "TupleOperation.hpp"
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk {
TupleOperation::TupleOperation(Mesh& m, const InvariantCollection& invariants, const Tuple& t)
    : Operation(m)
    , m_invariants(invariants)
    , m_input_tuple{t}
{}
TupleOperation::TupleOperation(Mesh& m, const InvariantCollection& invariants): TupleOperation(m,invariants,{})
{}
const Tuple& TupleOperation::input_tuple() const
{
    return m_input_tuple;
}

void TupleOperation::set_input_tuple(const Tuple& t) {
    m_input_tuple = t;
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

