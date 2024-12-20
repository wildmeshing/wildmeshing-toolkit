#pragma once

#include "Invariant.hpp"


#include <memory>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Rational.hpp>

namespace fastEnvelope {
class FastEnvelope;
}
namespace SimpleBVH {
class BVH;
}

namespace wmtk::invariants {

class EnvelopeInvariant : public Invariant
{
public:
    EnvelopeInvariant(
        const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
        double envelope_size,
        const attribute::MeshAttributeHandle& coordinate);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

    const std::shared_ptr<SimpleBVH::BVH>& bvh() const { return m_bvh; }
    const attribute::MeshAttributeHandle& coordinate_handle() const { return m_coordinate_handle; }


private:
    bool has_envelope() const;
    std::shared_ptr<fastEnvelope::FastEnvelope> m_envelope = nullptr;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
    const attribute::MeshAttributeHandle m_coordinate_handle;

    const double m_envelope_size;
};
} // namespace wmtk::invariants
