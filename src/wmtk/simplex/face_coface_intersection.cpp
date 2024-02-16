#include "face_coface_intersection.hpp"

#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::simplex {

std::vector<Tuple> face_coface_intersection(
    const Mesh& mesh,
    const Tuple& t,
    const PrimitiveType simplex_ptype,
    const PrimitiveType face_ptype)
{
    std::vector<PrimitiveType> switch_tuple_types =
        wmtk::utils::primitive_range(simplex_ptype, face_ptype);

    Tuple t_iter = t;

    std::vector<Tuple> intersection_tuples;
    do {
        intersection_tuples.emplace_back(t_iter);
        for (size_t i = 1; i < switch_tuple_types.size() - 1; ++i) {
            t_iter = mesh.switch_tuple(t_iter, switch_tuple_types[i]);
        }
    } while (t != t_iter);

    return intersection_tuples;
}

} // namespace wmtk::simplex