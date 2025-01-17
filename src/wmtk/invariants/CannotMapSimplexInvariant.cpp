#include "CannotMapSimplexInvariant.hpp"
#include <wmtk/simplex/neighbors_single_dimension.hpp>

#include <wmtk/Mesh.hpp>

namespace wmtk::components::isotropic_remeshing::invariants {
CannotMapSimplexInvariant::CannotMapSimplexInvariant(
    const Mesh& parent_mesh,
    const Mesh& child_mesh,
    PrimitiveType pt)
    : Invariant(parent_mesh, true, false, false)
    , m_child_mesh(child_mesh)
    , m_mapped_simplex_type(pt)
{}
CannotMapSimplexInvariant::CannotMapSimplexInvariant(const Mesh& parent_mesh, const Mesh& child_mesh)
    : CannotMapSimplexInvariant(parent_mesh, child_mesh, child_mesh.top_simplex_type())
{}
bool CannotMapSimplexInvariant::before(const simplex::Simplex& s) const
{
    if (m_mapped_simplex_type == m_child_mesh.top_simplex_type()) {
        return before_same_dimension(s);
    }
    return before_default(s);
}
    bool CannotMapSimplexInvariant::before_same_dimension_vertex(const simplex::Simplex& s) const {
    assert(m_mapped_simplex_type == m_child_mesh.top_simplex_type());
    assert(m_mapped_simplex_type == PrimitiveType::Vertex);

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
        }

    }
    bool CannotMapSimplexInvariant::before_same_dimension_edge(const simplex::Simplex& s) const {
    assert(m_mapped_simplex_type == m_child_mesh.top_simplex_type());
    assert(m_mapped_simplex_type == PrimitiveType::Edge);
    }
bool CannotMapSimplexInvariant::before_same_dimension(const simplex::Simplex& s) const
{
    assert(m_mapped_simplex_type == m_child_mesh.top_simplex_type());
    switch (m_mapped_simplex_type) {
    case PrimitiveType::Vertex:
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

bool CannotMapSimplexInvariant::before_default(const simplex::Simplex& s) const
{
    auto childs = simplex::neighbors_single_dimension(mesh(), s, m_mapped_simplex_type);
    for (const auto& child : childs) {
        if (mesh().can_map(m_child_mesh, child)) {
            return false;
        }
    }
    return true;
}
} // namespace wmtk::components::isotropic_remeshing::invariants

