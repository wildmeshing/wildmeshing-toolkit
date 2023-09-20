#pragma once

#include "OpenStarIterable.hpp"

namespace wmtk::simplex {
OpenStarIterable open_star_iterable(const Mesh& mesh, const Simplex& simplex);
}