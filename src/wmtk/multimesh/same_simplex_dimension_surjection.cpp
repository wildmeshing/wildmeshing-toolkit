#include "same_simplex_dimension_surjection.hpp"
#include <numeric>
#include <wmtk/Mesh.hpp>


namespace wmtk::multimesh {
std::vector<std::array<Tuple, 2>> same_simplex_dimension_bijection(
    const Mesh& parent,
    const Mesh& child)
{
    PrimitiveType primitive_type = parent.top_simplex_type();
    int64_t size = parent.capacity(primitive_type);
#if !defined(NDEBUG)
    // the MultimEshManager::surjection checks this as well - but we need to check it here to do the
    // capacity check
    if (primitive_type != child.top_simplex_type()) {
        throw std::runtime_error(
            "Cannot use same_simplex_dimension_bijection on meshes with simplex dimensions");
    }
    // TODO: this code is not aware of the difference between the size of a mesh and the capcity of
    // an attribute
    if (size != child.capacity(primitive_type)) {
        throw std::runtime_error(
            "Cannot use same_simplex_dimension_bijection on meshes with different capacities");
    }
#endif
    std::vector<int64_t> ps(size);
    std::iota(ps.begin(), ps.end(), 0);
    return same_simplex_dimension_surjection(parent, child, ps);
}

std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<int64_t>& parent_simplices)
{
    return MultiMeshManager::same_simplex_dimension_surjection(parent, child, parent_simplices);
}
} // namespace wmtk::multimesh
