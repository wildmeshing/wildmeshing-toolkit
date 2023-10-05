#include "same_simplex_dimension_surjection.hpp"
#include <numeric>
#include <wmtk/Mesh.hpp>


namespace wmtk::multimesh {
std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child)
{
    PrimitiveType primitive_type = parent.top_simplex_type();
    assert(primitive_type == child.top_simplex_type());
    long size = parent.capacity(primitive_type);
    assert(size == child.capacity(primitive_type));
    std::vector<long> ps;
    ps.reserve(size);
    std::iota(ps.begin(), ps.end(), 0);
    return same_simplex_dimension_surjection(parent, child, ps);
}

std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<long>& parent_simplices)
{
    return MultiMeshManager::same_simplex_dimension_surjection(parent, child, parent_simplices);
}
} // namespace wmtk::multimesh
