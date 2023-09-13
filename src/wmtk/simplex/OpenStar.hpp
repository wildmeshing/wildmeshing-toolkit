#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
template <typename MeshT>
class OpenStar : public SimplexCollection
{
public:
    OpenStar(const MeshT& mesh);
};
} // namespace wmtk::simplex
