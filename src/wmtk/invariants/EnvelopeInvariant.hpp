#pragma once

#include "Invariant.hpp"

#include <memory>

namespace fastEnvelope {
class FastEnvelope;
}

namespace wmtk::invariants {
class EnvelopeInvariant : public Invariant
{
public:
    EnvelopeInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& coordinate,
        const Mesh& envelope_mesh,
        const TypedAttributeHandle<double>& envelope_mesh_coordinate,
        double envelope_size);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<fastEnvelope::FastEnvelope> m_envelope;
    const TypedAttributeHandle<double> m_coordinate_handle;
};
} // namespace wmtk::invariants