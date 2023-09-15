#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
class SimplexBoundary : public SimplexCollection
{
public:
    SimplexBoundary(const Mesh& mesh, const Simplex& simplex, const bool sort = true);

private:
};
} // namespace wmtk::simplex
