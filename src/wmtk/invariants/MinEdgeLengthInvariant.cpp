
#include "MinEdgeLengthInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
MinEdgeLengthInvariant::MinEdgeLengthInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& coordinate,
    double threshold_squared)
    : MeshInvariant(m)
    , m_coordinate_handle(coordinate)
    , m_threshold_squared(threshold_squared)
{}
bool MinEdgeLengthInvariant::before(const Tuple& t) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);

    auto p0 = accessor.const_vector_attribute(t);
    auto p1 = accessor.const_vector_attribute(mesh().switch_vertex(t));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared > m_threshold_squared;
}
} // namespace wmtk

