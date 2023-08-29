#include "InteriorEdgeInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
bool InteriorEdgeInvariant::before(const Tuple& t) const
{
    const bool result = !mesh().is_boundary_edge(t);
    return result;
}
} // namespace wmtk
