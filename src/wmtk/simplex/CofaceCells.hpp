#pragma once

#include "SimplexCollection.hpp"

namespace wmtk {
class TriMesh;
class TetMesh;

namespace simplex {
class CofaceCells : public SimplexCollection
{
public:
    CofaceCells(const TriMesh& mesh, const Simplex& simplex, const bool sort = true);
    CofaceCells(const TetMesh& mesh, const Simplex& simplex, const bool sort = true);
};
} // namespace simplex
} // namespace wmtk