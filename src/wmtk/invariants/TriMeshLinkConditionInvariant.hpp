#pragma once

#include "TriMeshInvariant.hpp"

namespace wmtk {
class TriMeshLinkConditionInvariant : public TriMeshInvariant
{
    using TriMeshInvariant::TriMeshInvariant;
    bool before(const Simplex& t) const override;
};
} // namespace wmtk
