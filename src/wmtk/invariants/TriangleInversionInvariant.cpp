
#include "TriangleInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>

namespace wmtk {
TriangleInversionInvariant::TriangleInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& uv_coordinate)
    : MeshInvariant(m)
    , m_uv_coordinate_handle(uv_coordinate)
{}
bool TriangleInversionInvariant::before(const Tuple& t) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_uv_coordinate_handle);

    Eigen::Vector2d p0 = accessor.const_vector_attribute(t);
    Eigen::Vector2d p1 = accessor.const_vector_attribute(mesh().switch_vertex(t));
    Eigen::Vector2d p2 =
        accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(t)));

    return (triangle_2d_area<double>(p0, p1, p2) < 0);
}
} // namespace wmtk
