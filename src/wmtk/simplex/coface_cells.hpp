#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {

SimplexCollection
coface_cells(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
coface_cells(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
coface_cells(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

} // namespace wmtk::simplex