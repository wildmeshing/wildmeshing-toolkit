#pragma once

#include "SimplexCollection.hpp"

namespace wmtk {
    class Mesh;
    class TriMesh;
    class TetMesh;
}
namespace wmtk::simplex {
SimplexCollection
link(const Mesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
link(const TriMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
link(const TetMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
link_slow(const Mesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean = true);
} // namespace wmtk::simplex
