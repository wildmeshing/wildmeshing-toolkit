#include "EnvelopeInvariant.hpp"

#include <fastenvelope/FastEnvelope.h>


namespace wmtk::invariants {
EnvelopeInvariant::EnvelopeInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate,
    const Mesh& envelope_mesh,
    const TypedAttributeHandle<double>& envelope_mesh_coordinate,
    double envelope_size)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    m_envelope = std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    // bool is_outside(const std::array<Vector3, 3> &triangle) const;

    return true;
}


} // namespace wmtk::invariants