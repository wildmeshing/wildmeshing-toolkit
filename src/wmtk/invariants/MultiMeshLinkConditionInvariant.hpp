#pragma once

#include "MeshInvariant.hpp"

namespace wmtk {
class MultiMeshLinkConditionInvariant : public Invariant
{
public:
    MultiMeshLinkConditionInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace wmtk
