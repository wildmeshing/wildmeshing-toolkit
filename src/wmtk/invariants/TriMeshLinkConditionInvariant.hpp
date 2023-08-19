#pragma once

#include "TriMeshInvariant.hpp"

namespace wmtk {
class TriMeshLinkConditionInvariant : public TriMeshInvariant
{
    using TriMeshInvariant::TriMeshInvariant;
    bool before(const Tuple& t) const override;
};
} // namespace wmtk
