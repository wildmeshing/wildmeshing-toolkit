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
    PrimitiveType primitive_type = parent.top_simplex_type();
    assert(primitive_type == child.top_simplex_type());

    long size = child.capacity(primitive_type);
    assert(size == long(parent_simplices.size()));
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(size);

    auto parent_flag_accessor = parent.get_const_flag_accessor(primitive_type);
    auto child_flag_accessor = child.get_const_flag_accessor(primitive_type);

    for (long index = 0; index < size; ++index) {
        const Tuple ct = child.tuple_from_id(primitive_type, index);
        const Tuple pt = parent.tuple_from_id(primitive_type, parent_simplices.at(index));
        if (parent_flag_accessor.const_scalar_attribute(pt) & 1 == 0) {
            continue;
        }
        if (child_flag_accessor.const_scalar_attribute(pt) & 1 == 0) {
            continue;
        }

        ret.emplace_back(std::array<Tuple, 2>{{pt, ct}});
    }
    return ret;
}
} // namespace wmtk::multimesh
