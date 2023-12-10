#include "SubstructureTopologyPreservingInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
SubstructureTopologyPreservingInvariant::SubstructureTopologyPreservingInvariant(const Mesh& m)
    : MeshInvariant(m)
{}

bool SubstructureTopologyPreservingInvariant::before(const Tuple& t) const
{
    // TODO implement invariant...
    return false;
}

} // namespace wmtk::invariants