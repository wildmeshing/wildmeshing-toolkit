#pragma once

#include "SimplexCollection.hpp"

namespace wmtk {
class TriMesh;
class TetMesh;

namespace simplex {
class OpenStar : public SimplexCollection
{
public:
    OpenStar(const Mesh& mesh, const Simplex& simplex, const bool sort = true);
};
} // namespace simplex
} // namespace wmtk
