#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>

namespace wmtk::components::multimesh {


void from_facet_surjection(Mesh& parent, Mesh& child, const std::vector<int64_t>& parent_simplices)
{
    auto child_map =
        wmtk::multimesh::same_simplex_dimension_surjection(parent, child, parent_simplices);
    parent.register_child_mesh(child.shared_from_this(), child_map);
}

} // namespace wmtk::components::multimesh
