#pragma once

#include "iterable/LinkSingleDimensionIterable.hpp"

namespace wmtk::simplex {
LinkSingleDimensionIterable link_single_dimension_iterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType link_type);
}