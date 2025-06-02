#pragma once

#include "FusionEdgeInvariant.hpp"
#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class FusionEdgeInvariant : public Invariant
{
public:
    FusionEdgeInvariant(const Mesh& position_mesh, const Mesh& periodic_mesh);

    bool before(const simplex::Simplex& s) const override;

private:
    // const Mesh& m_periodic_mesh;
};
} // namespace invariants
} // namespace wmtk
