#include "SimplexInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "predicates.h"

namespace wmtk {
SimplexInversionInvariant::SimplexInversionInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& coordinate)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{}

bool SimplexInversionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);

    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(t);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(mesh().switch_vertex(t));
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(t)));
            Eigen::Vector3d p3 = accessor.const_vector_attribute(
                mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(t))));

            if (mesh().is_ccw(t)) {
                if (orient3d(p3.data(), p0.data(), p1.data(), p2.data()) <= 0) return false;

            } else {
                if (orient3d(p3.data(), p0.data(), p2.data(), p1.data()) <= 0) return false;
            }
        }


        return true;

    } else if (mesh().top_simplex_type() == PrimitiveType::Face) {
        assert(accessor.dimension() == 2);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            Tuple ccw_tuple = tuple;
            if (!mesh().is_ccw(tuple)) {
                ccw_tuple = mesh().switch_vertex(tuple);
            }
            Eigen::Vector2d p0 = accessor.const_vector_attribute(ccw_tuple);
            Eigen::Vector2d p1 = accessor.const_vector_attribute(mesh().switch_vertex(ccw_tuple));
            Eigen::Vector2d p2 = accessor.const_vector_attribute(
                mesh().switch_vertex(mesh().switch_edge(ccw_tuple)));

            if (orient2d(p0.data(), p1.data(), p2.data()) <= 0) return false;
        }


        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            double p0 = accessor.const_scalar_attribute(tuple);
            double p1 = accessor.const_scalar_attribute(mesh().switch_vertex(tuple));

            if (p0 > p1) return false;
        }

        return true;
    }


    return true;
}


} // namespace wmtk