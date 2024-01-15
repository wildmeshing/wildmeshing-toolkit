#include "NoBoundaryCollapseToInteriorInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
NoBoundaryCollapseToInteriorInvariant::NoBoundaryCollapseToInteriorInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}

bool NoBoundaryCollapseToInteriorInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    bool v0_on_boundary = mesh().is_boundary(PrimitiveType::Vertex, t.tuple());
    bool v1_on_boundary =
        mesh().is_boundary(PrimitiveType::Vertex, mesh().switch_vertex(t.tuple()));

    if (v0_on_boundary == v1_on_boundary) {
        return true;
    }

    if (v0_on_boundary) return false;
    return true;
}
} // namespace wmtk::invariants
