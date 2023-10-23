#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
// returns all of the boundary simplices of a simplex (i.e k-1 simplices that bound a k-simplex)
// This does not include itself
SimplexCollection
boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
} // namespace wmtk::simplex
