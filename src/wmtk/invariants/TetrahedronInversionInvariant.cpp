#include "TetrahedronInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include "predicates.h"

namespace wmtk {
TetrahedronInversionInvariant::TetrahedronInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& coordinate)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{}

bool TetrahedronInversionInvariant::after(
    const simplex::Simplex& input_simplex,
    PrimitiveType type,
    const std ::vector<Tuple>& tets) const
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

        if (mesh().is_ccw(t)) {
            if (orient3d(p3.data(), p0.data(), p1.data(), p2.data()) < 0) return false;

        } else {
            if (orient3d(p3.data(), p0.data(), p2.data(), p1.data()) < 0) return false;
        }
    }
    return true;
}


} // namespace wmtk