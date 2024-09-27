#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

namespace wmtk::components::multimesh {


void from_facet_bijection(Mesh& parent, Mesh& child)
{
    auto child_map = wmtk::multimesh::same_simplex_dimension_bijection(parent, child);
    parent.register_child_mesh(child.shared_from_this(), child_map);
}

} // namespace wmtk::components::multimesh
