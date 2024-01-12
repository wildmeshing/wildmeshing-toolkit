#include "neighbors_single_dimension.hpp"
#include "cofaces_single_dimension.hpp"
#include "faces_single_dimension.hpp"
#include "utils/tuple_vector_to_homogeneous_simplex_vector.hpp"
namespace wmtk::simplex {

// returns the
std::vector<Simplex>
neighbors_single_dimension(const Mesh& m, const Simplex& s, const PrimitiveType pt)
{
    return utils::tuple_vector_to_homogeneous_simplex_vector(
        neighbors_single_dimension_tuples(m, s, pt),
        pt);
}
std::vector<Tuple>
neighbors_single_dimension_tuples(const Mesh& m, const Simplex& s, const PrimitiveType tpt)
{
    const PrimitiveType mypt = s.primitive_type();
    assert(m.top_simplex_type() >= mypt);
    assert(m.top_simplex_type() >= tpt);

    if (mypt < tpt) {
        return cofaces_single_dimension_tuples(m, s, tpt);
    } else if (mypt > tpt) {
        return faces_single_dimension_tuples(m, s, tpt);
    } else {
        return {s.tuple()};
    }
}
} // namespace wmtk::simplex
