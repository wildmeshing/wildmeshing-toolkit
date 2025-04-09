#pragma once

#include "SimplexCollection.hpp"
namespace wmtk {
    class PointMesh;
    class EdgeMesh;
    class TriMesh;
    class TetMesh;
}

namespace wmtk::simplex {
SimplexCollection link_single_dimension(
    const Mesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean = true);

SimplexCollection link_single_dimension(
    const TriMesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean = true);

SimplexCollection link_single_dimension(
    const TetMesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean = true);

SimplexCollection link_single_dimension_slow(
    const Mesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean = true);
} // namespace wmtk::simplex
