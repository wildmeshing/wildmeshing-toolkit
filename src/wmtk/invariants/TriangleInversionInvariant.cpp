
#include "TriangleInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk {
TriangleInversionInvariant::TriangleInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& uv_coordinate)
    : MeshInvariant(m)
    , m_uv_coordinate_handle(uv_coordinate)
{}
bool TriangleInversionInvariant::after(PrimitiveType type, const std::vector<Tuple>& t) const
{
    if (type != PrimitiveType::Face) return true;
    // assume conterclockwise
    ConstAccessor<double> accessor = mesh().create_accessor(m_uv_coordinate_handle);
    for (auto& tuple : t) {
        Eigen::Vector2d p0 = accessor.const_vector_attribute(tuple);
        Eigen::Vector2d p1 = accessor.const_vector_attribute(mesh().switch_vertex(tuple));
        Eigen::Vector2d p2 =
            accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(tuple)));

        if (wmtk::utils::triangle_signed_2d_area(p0, p1, p2) < 0) return false;
    }
    return true;
}
} // namespace wmtk
