
#include "TupleOperation.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk::operations {
TupleOperation::TupleOperation(std::shared_ptr<InvariantCollection> invariants, const Simplex& t)
    : m_invariants(invariants)
    , m_input_simplex{t}
{}
// TupleOperation::TupleOperation(std::shared_ptr<InvariantCollection> invariants)
//     : TupleOperation(invariants, {})
//{}
const Tuple& TupleOperation::input_tuple() const
{
    return m_input_simplex.tuple();
}
const Simplex& TupleOperation::input_simplex() const
{
    return m_input_simplex;
}

void TupleOperation::set_input_simplex(const Simplex& t)
{
    m_input_simplex = t;
}

bool TupleOperation::before() const
{
    assert(invariants_pointer()); // check if invariants were created

    // check simplex validity in the operation mesh
    if (!base_mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = invariants().mesh();

    // TODO check if this is correct
    const std::vector<Simplex> invariant_simplices =
        base_mesh().map(invariants().mesh(), input_simplex());
    for (const Simplex& s : invariant_simplices) {
        if (!invariants().before(s)) {
            return false;
        }
    }
    return true;
}
bool TupleOperation::after() const
{
    return invariants().directly_modified_after(unmodified_primitives(), modified_primitives());
}
} // namespace wmtk::operations
