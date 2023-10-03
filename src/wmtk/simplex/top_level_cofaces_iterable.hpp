#pragma once

#include "iterable/TopLevelCofacesIterable.hpp"

namespace wmtk::simplex {
TopLevelCofacesIterable top_level_cofaces_iterable(const Mesh& mesh, const Simplex& simplex);
}