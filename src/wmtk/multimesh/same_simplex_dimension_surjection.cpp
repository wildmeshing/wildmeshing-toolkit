#include "same_simplex_dimension_surjection.hpp"
#include <numeric>
#include <wmtk/Mesh.hpp>


namespace wmtk::multimesh {

std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<int64_t>& parent_simplices)
{
    return MultiMeshManager::same_simplex_dimension_surjection(parent, child, parent_simplices);
}
} // namespace wmtk::multimesh
