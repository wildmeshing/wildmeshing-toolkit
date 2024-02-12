
#include "tuple_vector_to_homogeneous_simplex_vector.hpp"
#include <algorithm>
#include <iterator>

#include <wmtk/Mesh.hpp>

namespace wmtk::simplex::utils {
std::vector<Simplex> tuple_vector_to_homogeneous_simplex_vector(
    const Mesh& mesh,
    const std::vector<Tuple>& tups,
    PrimitiveType primitive)
{
    std::vector<Simplex> r;
    r.reserve(tups.size());
    std::transform(
        tups.begin(),
        tups.end(),
        std::back_inserter(r),
        [&mesh, primitive](const Tuple& t) { return Simplex(mesh, primitive, t); });
    return r;
}
} // namespace wmtk::simplex::utils
