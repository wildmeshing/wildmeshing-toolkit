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
        const Mesh& m,
        const TypedAttributeHandle<double>& coordinate,
        const TypedAttributeHandle<int64_t>& tag,
        int64_t value,
        const Eigen::MatrixXd& vertices,
        const Eigen::MatrixXi& faces,
        double envelope_size);

    EnvelopeInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& coordinate,
        const TypedAttributeHandle<int64_t>& tag,
        int64_t value,
        const Mesh& envelope_mesh,
        const TypedAttributeHandle<double>& envelope_mesh_coordinate,
        double envelope_size);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<fastEnvelope::FastEnvelope> m_envelope = nullptr;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
    const TypedAttributeHandle<double> m_coordinate_handle;
    const TypedAttributeHandle<int64_t> m_tag;
    const int64_t m_value;
    const double m_envelope_size;
};
} // namespace wmtk::invariants