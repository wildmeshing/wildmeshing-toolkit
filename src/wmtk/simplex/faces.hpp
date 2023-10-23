#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
// returns all simplices that lie on the boundary of the input simplex (i.e all cofaces)
// This does not include itself
SimplexCollection faces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
} // namespace wmtk::simplex
