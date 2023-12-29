
#include "TriangleInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk {
TriangleInversionInvariant::TriangleInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& uv_coordinate)
    : Invariant(m)
    , m_uv_coordinate_handle(uv_coordinate)
{}
bool TriangleInversionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_uv_coordinate_handle);
    // assume conterclockwise
    for (const Tuple& tuple : top_dimension_tuples_after) {
        Tuple ccw_tuple = tuple;
        if (!mesh().is_ccw(tuple)) {
            ccw_tuple = mesh().switch_vertex(tuple);
        }
        Eigen::Vector2d p0 = accessor.const_vector_attribute(ccw_tuple);
        Eigen::Vector2d p1 = accessor.const_vector_attribute(mesh().switch_vertex(ccw_tuple));
        Eigen::Vector2d p2 =
            accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(ccw_tuple)));

        if (wmtk::utils::triangle_signed_2d_area(p0, p1, p2) <= 0) return false;
    }
    return true;
}
} // namespace wmtk
