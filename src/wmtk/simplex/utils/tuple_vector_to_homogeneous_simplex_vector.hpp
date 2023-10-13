#pragma once
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>


namespace wmtk::simplex::utils {
std::vector<Simplex> tuple_vector_to_homogeneous_simplex_vector(
    const std::vector<Tuple>&,
    PrimitiveType primitive);
}
