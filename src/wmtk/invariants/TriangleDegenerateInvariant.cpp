
#include "TriangleDegenerateInvariant.hpp"
#include <predicates.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/triangle_areas.hpp>
namespace wmtk {
TriangleDegenerateInvariant::TriangleDegenerateInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& uv_coordinate)
    : Invariant(m)
    , m_uv_coordinate_handle(uv_coordinate)
{}
bool TriangleDegenerateInvariant::after(PrimitiveType type, const std::vector<Tuple>& t) const
{
    if (type != PrimitiveType::Face) return true;
    ConstAccessor<double> accessor = mesh().create_accessor(m_uv_coordinate_handle);
    // assume conterclockwise
    for (auto& tuple : t) {
        Tuple ccw_tuple = tuple;
        if (!mesh().is_ccw(tuple)) {
            ccw_tuple = mesh().switch_vertex(tuple);
        }
        Eigen::Vector3d p0 = accessor.const_vector_attribute(ccw_tuple);
        Eigen::Vector3d p1 = accessor.const_vector_attribute(mesh().switch_vertex(ccw_tuple));
        Eigen::Vector3d p2 =
            accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(ccw_tuple)));

        if (abs(wmtk::utils::triangle_3d_area(p0, p1, p2)) <=
                std::numeric_limits<double>::denorm_min() ||
            abs(wmtk::utils::triangle_3d_area(p1, p2, p0)) <=
                std::numeric_limits<double>::denorm_min() ||
            abs(wmtk::utils::triangle_3d_area(p2, p0, p1)) <=
                std::numeric_limits<double>::denorm_min())
            return false;
    }
    return true;
}
} // namespace wmtk
