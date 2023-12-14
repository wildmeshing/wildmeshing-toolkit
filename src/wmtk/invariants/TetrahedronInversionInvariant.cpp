#include "TetrahedronInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include "predicates.h"

namespace wmtk {
TetrahedronInversionInvariant::TetrahedronInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& coordinate)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{}

bool TetrahedronInversionInvariant::after(PrimitiveType type, const std ::vector<Tuple>& tets) const
{
    if (type != PrimitiveType::Tetrahedron) return true;
    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);
    for (const auto& t : tets) {
        Eigen::Vector3d p0 = accessor.const_vector_attribute(t);
        Eigen::Vector3d p1 = accessor.const_vector_attribute(mesh().switch_vertex(t));
        Eigen::Vector3d p2 =
            accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(t)));
        Eigen::Vector3d p3 = accessor.const_vector_attribute(
            mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(t))));

        // ccw tuple have the inverted orientation of local v0, v1, v2, v3
        // we assume cw to have cw volume
        if (!mesh().is_ccw(t)) {
            // std::cout << "ccw tuple: o3d = " << orient3d(p0.data(), p1.data(), p2.data(),
            // p3.data())
            //           << std::endl;
            if (orient3d(p0.data(), p1.data(), p2.data(), p3.data()) < 0) {
                // std::cout << "false" << std::endl;
                return false;
            }
        } else {
            // std::cout << "not ccw tuple: o3d = "
            //           << orient3d(p1.data(), p0.data(), p2.data(), p3.data()) << std::endl;
            if (orient3d(p1.data(), p0.data(), p2.data(), p3.data()) < 0) {
                // std::cout << "false" << std::endl;
                return false;
            }
        }
    }
    return true;
}


} // namespace wmtk