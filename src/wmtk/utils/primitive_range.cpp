#include "primitive_range.hpp"
namespace wmtk::utils {
std::vector<PrimitiveType> primitive_range(PrimitiveType pt0, PrimitiveType pt1)
{
    std::vector<PrimitiveType> r;
    long start = get_primitive_type_id(pt0);
    long end = get_primitive_type_id(pt1);
    if (start < end) {
        r.reserve(end - start);
        for (long j = start; j <= end; ++j) {
            r.emplace_back(get_primitive_type_from_id(j));
        }
    } else {
        r.reserve(start - end);
        for (long j = start; j >= end; --j) {
            r.emplace_back(get_primitive_type_from_id(j));
        }
    }
    return r;
}
std::vector<PrimitiveType> primitive_above(PrimitiveType pt, bool lower_to_upper)
{
    if (lower_to_upper) {
        return primitive_range(pt, PrimitiveType::Tetrahedron);
    } else {
        return primitive_range(PrimitiveType::Tetrahedron, pt);
    }
}
std::vector<PrimitiveType> primitive_below(PrimitiveType pt, bool lower_to_upper)
{
    if (lower_to_upper) {
        return primitive_range(PrimitiveType::Vertex, pt);
    } else {
        return primitive_range(pt, PrimitiveType::Vertex);
    }
}
} // namespace wmtk::utils
