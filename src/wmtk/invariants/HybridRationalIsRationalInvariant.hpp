#pragma once
#include <wmtk/attribute/utils/HybridRationalAttribute.hpp>

#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class HybridRationalIsRationalInvariant: public Invariant
{
public:
    HybridRationalIsRationalInvariant(const Mesh& m, const attribute::utils::HybridRationalAttribute<>& coordinate);
    HybridRationalIsRationalInvariant(const attribute::MeshAttributeHandle& m);
    using Invariant::Invariant;

    /**
     *  we assume with local vid order (v0,v1,v2,v3) has positive volume (orient3d(v0,v1,v2,v3)>0)
     a CLOCKWISE input tuple will have the same orientation as the local vid order
     */
    bool before(const simplex::Simplex& t)
        const override;

private:
    const attribute::MeshAttributeHandle m_coordinate_handle;
    // const TypedAttributeHandle<double> m_coordinate_handle;
};
} // namespace wmtk
