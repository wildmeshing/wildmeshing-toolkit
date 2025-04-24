#pragma once

#include "SimplexCollection.hpp"

namespace wmtk {
    class TriMesh;
    class TetMesh;
}
namespace wmtk::simplex {
SimplexCollection
open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
open_star(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
open_star(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
open_star_slow(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
} // namespace wmtk::simplex
