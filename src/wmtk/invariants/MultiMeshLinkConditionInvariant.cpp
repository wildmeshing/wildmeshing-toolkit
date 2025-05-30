#include "MultiMeshLinkConditionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/link_condition.hpp>

namespace wmtk::invariants {

MultiMeshLinkConditionInvariant::MultiMeshLinkConditionInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool MultiMeshLinkConditionInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);

    return simplex::link_condition(mesh(), t.tuple());
}
} // namespace wmtk::invariants
