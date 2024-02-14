#pragma once
#include <vector>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::simplex {
class Simplex;

// computes every simplex of the target type that is a face or coface of the input simplex
std::vector<Simplex> neighbors_single_dimension(
    const Mesh& m,
    const Simplex& input_simplex,
    const PrimitiveType target_primitive_type);
std::vector<Tuple> neighbors_single_dimension_tuples(
    const Mesh& m,
    const Simplex& input_simplex,
    const PrimitiveType target_primitive_type);
} // namespace wmtk::simplex
