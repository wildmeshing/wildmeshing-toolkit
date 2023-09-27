#include "InteriorVertexInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
bool InteriorVertexInvariant::before(const Tuple& t) const
{
    const bool result = !mesh().is_boundary_vertex(t);
    return result;
}
} // namespace wmtk
