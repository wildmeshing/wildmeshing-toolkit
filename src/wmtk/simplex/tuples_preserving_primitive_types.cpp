#include "tuples_preserving_primitive_types.hpp"

#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::simplex {

std::vector<Tuple> tuples_preserving_primitive_types(
    const Mesh& mesh,
    const Tuple& t,
    const PrimitiveType simplex_ptype,
    const PrimitiveType face_ptype)
{
    // old and slow implementation

    const std::vector<PrimitiveType> switch_tuple_types =
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

void simplices_preserving_primitive_types(
    SimplexCollection& collection,
    const Tuple& t,
    const PrimitiveType simplex_ptype,
    const PrimitiveType face_ptype,
    const PrimitiveType pt_return)
{
    if (simplex_ptype == pt_return) {
        collection.add(pt_return, t);
        return;
    }

    const Mesh& mesh = collection.mesh();

    const int8_t pt0_id = get_primitive_type_id(face_ptype);
    const int8_t pt1_id = get_primitive_type_id(simplex_ptype);
    assert(pt0_id <= pt1_id);

    Tuple t_iter = t;

    do {
        collection.add(pt_return, t_iter);
        for (int8_t i = pt0_id + 1; i < pt1_id; ++i) {
            const PrimitiveType pt = get_primitive_type_from_id(i);
            t_iter = mesh.switch_tuple(t_iter, pt);
        }
    } while (t != t_iter);
}

} // namespace wmtk::simplex