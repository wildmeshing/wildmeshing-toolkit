#pragma once

#include "FusionEdgeInvariant.hpp"
#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class uvEdgeInvariant : public Invariant
{
public:
    uvEdgeInvariant(const Mesh& position_mesh, const Mesh& uv_mesh);

    bool before(const simplex::Simplex& s) const override;

private:
    // const Mesh& m_uv_mesh;
};
} // namespace invariants
} // namespace wmtk
