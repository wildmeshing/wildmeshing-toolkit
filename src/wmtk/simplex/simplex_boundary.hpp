#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
[[deprecated]] SimplexCollection
simplex_boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
}
