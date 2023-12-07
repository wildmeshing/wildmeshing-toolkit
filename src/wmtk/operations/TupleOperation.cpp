
#include "TupleOperation.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk::operations {
TupleOperation::TupleOperation(std::shared_ptr<InvariantCollection> invariants, const Tuple& t)
    : m_invariants(invariants)
    , m_input_tuple{t}
{}
TupleOperation::TupleOperation(std::shared_ptr<InvariantCollection> invariants)
    : TupleOperation(invariants, {})
{}
const Tuple& TupleOperation::input_tuple() const
{
    return m_input_tuple;
}

void TupleOperation::set_input_tuple(const Tuple& t)
{
    m_input_tuple = t;
}

bool TupleOperation::before() const
{
    MeshInvariant::m_mesh = &(base_mesh()); // TODO HACK remove this!!!

    // check tuple validity in the operation mesh
    base_mesh().is_valid_slow(input_tuple());
    // map tuple to the invariant mesh
    const Mesh& invariant_mesh = invariants().mesh();
    // Tuple invariant_tuple = invariant_mesh.map(base_mesh(), input_tuple()); // TODO HACK
    Tuple invariant_tuple = input_tuple(); // TODO HACK
    return invariants().before(invariant_tuple);
}
bool TupleOperation::after() const
{
    return invariants().after(PrimitiveType::Face, modified_primitives(PrimitiveType::Face));
}
std::vector<Tuple> TupleOperation::modified_primitives(PrimitiveType) const
{
    return {};
}
} // namespace wmtk::operations
