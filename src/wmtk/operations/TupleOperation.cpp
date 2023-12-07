
#include "TupleOperation.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk::operations {
TupleOperation::TupleOperation(std::shared_ptr<InvariantCollection> invariants, const Simplex& t)
    : m_invariants(invariants)
    , m_input_simplex{t}
{
}
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
    MeshInvariant::m_mesh = &(base_mesh()); // TODO HACK remove this!!!

    assert(invariants_pointer()); // check if invariants were created

    // check simplex validity in the operation mesh
    if (!base_mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = invariants().mesh();

    spdlog::warn("About to start before invars");
    // TODO check if this is correct
    const std::vector<Simplex> invariant_simplices =
        base_mesh().map(invariants().mesh(), input_simplex());
    for (const Simplex& s : invariant_simplices) {
        if (!invariants().before(s)) {
            spdlog::info("before failed on invar");
            return false;
        }
    }
            spdlog::info("before succeeded on invar");
    return true;
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
