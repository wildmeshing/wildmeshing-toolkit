#include "NoChildSimplexInvariant.hpp"
#include <wmtk/simplex/neighbors_single_dimension.hpp>

#include <wmtk/Mesh.hpp>

namespace wmtk::components::isotropic_remeshing::invariants {
NoChildSimplexInvariant::NoChildSimplexInvariant(
    const Mesh& parent_mesh,
    const Mesh& child_mesh,
    PrimitiveType pt)
    : Invariant(parent_mesh, true, false, false)
    , m_child_mesh(child_mesh)
    , m_primitive_type(pt)
{}
NoChildSimplexInvariant::NoChildSimplexInvariant(const Mesh& parent_mesh, const Mesh& child_mesh)
    : NoChildSimplexInvariant(parent_mesh, child_mesh, child_mesh.top_simplex_type())
{}
bool NoChildSimplexInvariant::before(const simplex::Simplex& s) const
{
    if (m_primitive_type == m_child_mesh.top_simplex_type()) {
        return before_same(s);
    } else {
        return before_lower(s);
    }
}
bool NoChildSimplexInvariant::before_same(const simplex::Simplex& s) const
{
    switch (m_primitive_type) {
    case PrimitiveType::Vertex:
        switch (s.primitive_type()) {
        case PrimitiveType::Vertex: return !mesh().can_map(m_child_mesh, s);
        case PrimitiveType::Edge: {
            return !(
                mesh().can_map(m_child_mesh, simplex::Simplex(PrimitiveType::Vertex, s.tuple())) ||
                mesh().can_map(
                    m_child_mesh,
                    simplex::Simplex(
                        PrimitiveType::Vertex,
                        mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex))));
        }
        case PrimitiveType::Triangle:
        default:
        case PrimitiveType::Tetrahedron:
            break;
            {
                simplex::Simplex s1(PrimitiveType::Vertex, s.tuple());
                simplex::Simplex s2(
                    PrimitiveType::Vertex,
                    mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex));
                return !mesh().can_map(m_child_mesh, s1) && !mesh().can_map(m_child_mesh, s2);
            }
        }
        break;
    case PrimitiveType::Edge:
        switch (m_child_mesh.top_simplex_type()) {
        case PrimitiveType::Vertex:
        case PrimitiveType::Edge: return !mesh().can_map(m_child_mesh, s);
        case PrimitiveType::Triangle:
        default:
        case PrimitiveType::Tetrahedron: break;
        }
    case PrimitiveType::Triangle:
    default:
    case PrimitiveType::Tetrahedron: break;
    }
    assert(false); // not implemented
    return false;
}

bool NoChildSimplexInvariant::before_lower(const simplex::Simplex& s) const
{
    auto childs = simplex::neighbors_single_dimension(mesh(), s, m_primitive_type);
    for (const auto& child : childs) {
        if (mesh().can_map(m_child_mesh, child)) {
            return false;
        }
    }
    return true;
}
} // namespace wmtk::components::isotropic_remeshing::invariants

