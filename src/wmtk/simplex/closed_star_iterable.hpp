#pragma once

#include "iterable/ClosedStarIterable.hpp"

namespace wmtk::simplex {
ClosedStarIterable closed_star_iterable(const Mesh& mesh, const Simplex& simplex);
}