#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {

SimplexCollection
top_level_cofaces(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
top_level_cofaces(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
top_level_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

} // namespace wmtk::simplex