#pragma once

#include "MeshInvariant.hpp"

namespace wmtk {
class MultiMeshMapValidInvariant : public MeshInvariant
{
public:
    MultiMeshMapValidInvariant(const Mesh& m);
    bool before(const Tuple& t) const override;
};
} // namespace wmtk
