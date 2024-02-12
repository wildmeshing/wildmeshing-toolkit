#pragma once
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>


namespace wmtk::simplex::utils {
std::vector<Simplex> tuple_vector_to_homogeneous_simplex_vector(
    const Mesh& mesh,
    const std::vector<Tuple>&,
    PrimitiveType primitive);

} // namespace wmtk::simplex::utils