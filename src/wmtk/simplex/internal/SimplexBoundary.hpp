#pragma once

#include <wmtk/simplex/SimplexCollection.hpp>

namespace wmtk::simplex::internal {
class SimplexBoundary : public SimplexCollection
{
public:
    SimplexBoundary(const Mesh& mesh, const Simplex& simplex, const bool sort = true);

private:
};
} // namespace wmtk::simplex::internal
