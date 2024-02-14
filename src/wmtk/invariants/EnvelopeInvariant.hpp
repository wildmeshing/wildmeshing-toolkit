#pragma once

#include "Invariant.hpp"

#include <Eigen/Dense>

#include <memory>
#include <wmtk/attribute/AttributeHandle.hpp>

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

private:
    std::shared_ptr<fastEnvelope::FastEnvelope> m_envelope = nullptr;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
    const TypedAttributeHandle<double> m_coordinate_handle;
    const double m_envelope_size;
};
} // namespace wmtk::invariants