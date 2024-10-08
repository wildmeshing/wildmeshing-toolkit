#include "Swap2dEdgeLengthInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/utils/orient.hpp>

namespace wmtk {
Swap2dEdgeLengthInvariant::Swap2dEdgeLengthInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<double>& coordinate)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
{}

bool Swap2dEdgeLengthInvariant::before(const simplex::Simplex& t) const
{
    assert(mesh().top_cell_dimension() == 2);
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);

    // get the coords of the vertices
    // input face end points
    const Tuple v0 = t.tuple();
    const Tuple v1 = mesh().switch_tuple(v0, PV);
    // other 2 vertices
    const Tuple v2 = mesh().switch_tuples(v0, {PE, PV});
    const Tuple v3 = mesh().switch_tuples(v0, {PF, PE, PV});

    // use length
    // const double length_old =
    //     (accessor.const_vector_attribute(v0) - accessor.const_vector_attribute(v1)).norm();
    // const double length_new =
    //     (accessor.const_vector_attribute(v2) - accessor.const_vector_attribute(v3)).norm();

    // return length_new > length_old;

    // use height

    Eigen::Vector3d p0 = accessor.const_vector_attribute(v0);
    Eigen::Vector3d p1 = accessor.const_vector_attribute(v1);
    Eigen::Vector3d p2 = accessor.const_vector_attribute(v2);
    Eigen::Vector3d p3 = accessor.const_vector_attribute(v3);

    const double h0 = ((p0 - p2).cross(p0 - p3)).norm() / (p2 - p3).norm();
    const double h1 = ((p1 - p2).cross(p1 - p3)).norm() / (p2 - p3).norm();
    const double h2 = ((p2 - p0).cross(p2 - p1)).norm() / (p0 - p1).norm();
    const double h3 = ((p3 - p0).cross(p3 - p1)).norm() / (p0 - p1).norm();

    const double min_old = std::min(h2, h3);
    const double min_new = std::min(h0, h1);

    if (((p0 - p1).norm() * h2 / 2) < 1e-10 || ((p0 - p1).norm() * h3 / 2) < 1e-10) {
        return true;
    }

    return min_old < min_new;
}

} // namespace wmtk
