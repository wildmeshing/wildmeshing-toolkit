
#include "tuple_vector_to_homogeneous_simplex_vector.hpp"

#include <wmtk/Mesh.hpp>

#include <algorithm>
#include <iterator>

namespace wmtk::simplex::utils {
std::vector<Simplex> tuple_vector_to_homogeneous_simplex_vector(
    const Mesh& m,
    const std::vector<Tuple>& tups,
    PrimitiveType primitive)
{
    std::vector<Simplex> r;
    r.reserve(tups.size());
    std::transform(
        tups.begin(),
        tups.end(),
        std::back_inserter(r),
        [&m, primitive](const Tuple& t) { return Simplex(m, primitive, t); });
    return r;
}
std::vector<IdSimplex> tuple_vector_to_homogeneous_id_simplex_vector(
    const Mesh& m,
    const std::vector<Tuple>& tups,
    PrimitiveType primitive)
{
    std::vector<IdSimplex> r;
    r.reserve(tups.size());
    std::transform(
        tups.begin(),
        tups.end(),
        std::back_inserter(r),
        [&m, primitive](const Tuple& t) { return m.get_id_simplex(t, primitive); });
    return r;
}
} // namespace wmtk::simplex::utils
