#include "FrozenVertexInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
FrozenVertexInvariant::FrozenVertexInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& frozen_vertex_handle)
    : Invariant(m, true, false, false)
    , m_frozen_vertex_handle(frozen_vertex_handle)
{}

bool FrozenVertexInvariant::before(const simplex::Simplex& t) const
{
    const auto accessor = mesh().create_const_accessor(m_frozen_vertex_handle);

    if (accessor.const_scalar_attribute(simplex::Simplex::vertex(mesh(), t.tuple())) == 1) {
        return false;
    }
    return true;
}

FrozenOppVertexInvariant::FrozenOppVertexInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& frozen_vertex_handle)
    : Invariant(m, true, false, false)
    , m_frozen_vertex_handle(frozen_vertex_handle)
{}

bool FrozenOppVertexInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    const auto accessor = mesh().create_const_accessor(m_frozen_vertex_handle);

    if (accessor.const_scalar_attribute(simplex::Simplex::vertex(
            mesh(),
            mesh().switch_tuple(t.tuple(), PrimitiveType::Vertex))) == 1) {
        return false;
    }
    return true;
}
} // namespace wmtk::invariants
