#pragma once
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
class Mesh;
}

namespace wmtk::simplex::utils {

std::vector<Simplex> tuple_vector_to_homogeneous_simplex_vector(
    const Mesh& m,
    const std::vector<Tuple>& tups,
    PrimitiveType primitive);

std::vector<IdSimplex> tuple_vector_to_homogeneous_id_simplex_vector(
    const Mesh& m,
    const std::vector<Tuple>& tups,
    PrimitiveType primitive);

} // namespace wmtk::simplex::utils
