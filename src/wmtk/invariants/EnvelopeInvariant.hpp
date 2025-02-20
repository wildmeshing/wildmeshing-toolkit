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
    /**
     * @brief Creates an envelope for checking if vertices are inside or outside of it.
     *
     * The check is performed on the `coordinate`, the envelope is constructed from
     * `envelope_mesh_coordinate`.
     *
     * @param envelope_mesh_coordinate Used for constructing the envelope.
     * @param envelope_size
     * @param coordinate This position handle represents the mesh the envelope is tested on.
     */
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
    const attribute::MeshAttributeHandle m_coordinate_handle;

    const double m_envelope_size;
};
} // namespace wmtk::invariants
