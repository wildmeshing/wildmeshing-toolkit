#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::simplex {

    // Returns the cofaces of a provided simplex, but only providing the cofaces in the provided coface type

std::vector<Tuple> cofaces_single_dimension_tuples(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type);

std::vector<Tuple> cofaces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type);
} // namespace wmtk::simplex
