#include "TriMeshLinkConditionInvariant.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
bool TriMeshLinkConditionInvariant::before(const Tuple& t) const
{
    const bool result = SimplicialComplex::link_cond_bd_2d(mesh(), t);
    return result;
}
} // namespace wmtk
