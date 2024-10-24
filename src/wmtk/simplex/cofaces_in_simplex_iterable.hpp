#pragma once

#include "iterable/CofacesInSimplexIterable.hpp"

namespace wmtk::simplex {

CofacesInSimplexIterable cofaces_in_simplex_iterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType in_simplex_type);

} // namespace wmtk::simplex
