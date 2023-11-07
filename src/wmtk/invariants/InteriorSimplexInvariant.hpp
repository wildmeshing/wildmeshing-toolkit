#pragma once

#include "MeshInvariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorSimplexInvariant : public MeshInvariant
{
public:
    using MeshInvariant::MeshInvariant;
    InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt);
    bool before(const Tuple& t) const override;

private:
    PrimitiveType m_primitive_type;
};
} // namespace invariants
} // namespace wmtk
