#include "MaxEdgeLengthInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
    MaxEdgeLengthAttribute::MaxEdgeLengthInvariant(const Mesh& m, const MeshAttributeHandle<double>& coordinate, double threshold_squared): MeshInvariant(m), m_coordinate(coordinate), m_threshold_squared(threshold_squared) {}
bool MaxEdgeLengthInvariant::before(const Tuple& t) const
{
   ConstAccessor<double> accessor = mesh().get_accessor(m_coordinate_handle);

    auto p0 = m_pos_accessor.vector_attribute(input_tuple());
    auto p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(input_tuple()));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_threshold_squared;
}
} // namespace wmtk

