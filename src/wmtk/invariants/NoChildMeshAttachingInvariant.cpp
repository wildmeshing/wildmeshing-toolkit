#include "NoChildMeshAttachingInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
NoChildMeshAttachingInvariant::NoChildMeshAttachingInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}

bool NoChildMeshAttachingInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);

    auto child_meshes = mesh().get_child_meshes();

    for (auto child_mesh : child_meshes) {
        if (!mesh().map_to_child(*child_mesh, simplex::Simplex::edge(mesh(), t.tuple())).empty()) {
            continue;
        }
        if (!mesh()
                 .map_to_child(*child_mesh, simplex::Simplex::vertex(mesh(), t.tuple()))
                 .empty() ||
            !mesh()
                 .map_to_child(
                     *child_mesh,
                     simplex::Simplex::vertex(
                         mesh(),
                         mesh().switch_tuple(t.tuple(), PrimitiveType::Vertex)))
                 .empty()) {
            return false;
        }
    }

    return true;
}
} // namespace wmtk::invariants
