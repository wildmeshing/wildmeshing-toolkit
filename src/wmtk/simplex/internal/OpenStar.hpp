#pragma once

#include <wmtk/simplex/SimplexCollection.hpp>

namespace wmtk {
class TriMesh;
class TetMesh;

namespace simplex::internal {
class OpenStar : public SimplexCollection
{
public:
    OpenStar(const Mesh& mesh, const Simplex& simplex, const bool sort = true);
};
} // namespace simplex::internal
} // namespace wmtk
