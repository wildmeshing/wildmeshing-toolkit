#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
namespace wmtk::simplex {

// Returns the cofaces of a provided simplex, but only providing the cofaces in the provided coface
// type

SimplexCollection cofaces_single_dimension(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type, bool sort_and_clean=true);

std::vector<Tuple> cofaces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type);

std::vector<Simplex> cofaces_single_dimension_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type);

} // namespace wmtk::simplex
