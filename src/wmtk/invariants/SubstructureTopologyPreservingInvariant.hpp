#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk::invariants {

class SubstructureTopologyPreservingInvariant : public MeshInvariant
{
public:
    SubstructureTopologyPreservingInvariant(const Mesh& m);
    bool before(const Tuple& t) const override;

private:
};

} // namespace wmtk::invariants
