#pragma once

#include "iterable/TopDimensionCofacesIterable.hpp"

namespace wmtk::simplex {
TopDimensionCofacesIterable top_dimension_cofaces_iterable(
    const Mesh& mesh,
    const Simplex& simplex);
}