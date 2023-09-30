#pragma once

#include "iterable/CofaceCellsIterable.hpp"

namespace wmtk::simplex {
CofaceCellsIterable top_level_cofaces_iterable(const Mesh& mesh, const Simplex& simplex);
}