#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include "TriMeshLinkConditionInvariant.hpp"
#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {
bool TriMeshLinkConditionInvariant::before(const Tuple& t) const
{
    return SimplicialComplex::link_cond_bd_2d(mesh(), t);
}
} // namespace wmtk
