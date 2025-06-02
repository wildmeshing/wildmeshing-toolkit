#include "LinkConditionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/link_condition.hpp>

namespace wmtk::invariants {

LinkConditionInvariant::LinkConditionInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool LinkConditionInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);

    return simplex::link_condition(mesh(), t.tuple());
}
} // namespace wmtk::invariants
