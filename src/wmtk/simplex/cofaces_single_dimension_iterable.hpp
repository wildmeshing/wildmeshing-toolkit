#pragma once

#include "iterable/CofacesSingleDimensionIterable.hpp"

namespace wmtk::simplex {
CofacesSingleDimensionIterable cofaces_single_dimension_iterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType cofaces_type);
}