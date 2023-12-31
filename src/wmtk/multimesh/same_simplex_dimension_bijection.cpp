
#include "same_simplex_dimension_bijection.hpp"
#include <numeric>
#include <wmtk/Mesh.hpp>
#include "same_simplex_dimension_surjection.hpp"


namespace wmtk::multimesh {
std::vector<std::array<Tuple, 2>> same_simplex_dimension_bijection(
    const Mesh& parent,
    const Mesh& child)
{
    PrimitiveType primitive_type = parent.top_simplex_type();
    assert(primitive_type == child.top_simplex_type());
    int64_t size = parent.capacity(primitive_type);
#if !defined(NDEBUG)
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

} // namespace wmtk::multimesh
