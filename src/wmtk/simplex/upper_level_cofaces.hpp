#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::simplex {
std::vector<Tuple> upper_level_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    const PrimitiveType& cofaces_type);

std::vector<Tuple> upper_level_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    const PrimitiveType& cofaces_type);
} // namespace wmtk::simplex