#pragma once

#include "iterable/CofaceCellsIterable.hpp"

namespace wmtk::simplex {
CofaceCellsIterable coface_cells_iterable(const Mesh& mesh, const Simplex& simplex);
}