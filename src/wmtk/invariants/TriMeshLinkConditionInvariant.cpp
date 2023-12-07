#include "TriMeshLinkConditionInvariant.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
bool TriMeshLinkConditionInvariant::before(const Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);

    const bool result = SimplicialComplex::link_cond_bd_2d(mesh(), t.tuple());
    return result;
}
} // namespace wmtk
