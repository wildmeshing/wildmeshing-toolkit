#pragma once

#include <wmtk/simplex/SimplexCollection.hpp>

namespace wmtk {
class TriMesh;
class TetMesh;

namespace simplex::internal {
class CofaceCells : public SimplexCollection
{
public:
    CofaceCells(const TriMesh& mesh, const Simplex& simplex, const bool sort = true);
    CofaceCells(const TetMesh& mesh, const Simplex& simplex, const bool sort = true);
};
} // namespace simplex::internal
} // namespace wmtk