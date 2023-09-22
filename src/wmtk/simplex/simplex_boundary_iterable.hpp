#pragma once

#include "iterable/SimplexBoundaryIterable.hpp"

namespace wmtk::simplex {
SimplexBoundaryIterable simplex_boundary_iterable(const Mesh& mesh, const Simplex& simplex);
}