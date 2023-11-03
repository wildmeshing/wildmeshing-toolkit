#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
SimplexCollection
open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
}