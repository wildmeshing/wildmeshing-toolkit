#include "NoBoundaryCollapseToInteriorInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
NoBoundaryCollapseToInteriorInvariant::NoBoundaryCollapseToInteriorInvariant(const Mesh& m)
    : Invariant(m)
{}

bool NoBoundaryCollapseToInteriorInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    bool v0_on_boundary = mesh().is_boundary(t.tuple(), PrimitiveType::Vertex);
    bool v1_on_boundary =
        mesh().is_boundary(mesh().switch_vertex(t.tuple()), PrimitiveType::Vertex);

    if (v0_on_boundary == v1_on_boundary) {
        return true;
    }

    if (v0_on_boundary) return false;
    return true;
}
} // namespace wmtk::invariants