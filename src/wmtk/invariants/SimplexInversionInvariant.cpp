#include "SimplexInversionInvariant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include "internal/simplex_inversion_predicates.hpp"

namespace wmtk {
SimplexInversionInvariant::SimplexInversionInvariant(
    const attribute::MeshAttributeHandle& coordinate)
    : Invariant(coordinate.mesh(), true, false, true)
    , m_coordinate_handle(coordinate)
{}
SimplexInversionInvariant::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate)
    : SimplexInversionInvariant(attribute::MeshAttributeHandle(m, coordinate))
{}

bool SimplexInversionInvariant::after(
    const std::vector<Tuple>&,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor<3>(m_coordinate_handle);
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            const wmtk::Tuple t1 = mymesh.switch_tuple(t, PrimitiveType::Vertex);
            const wmtk::Tuple t2 =
                mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex});
            const wmtk::Tuple t3 = mymesh.switch_tuples(
                t,
                {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex});
            Eigen::Vector3d p0 = accessor.const_vector_attribute(t);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(t1);
            Eigen::Vector3d p2 = accessor.const_vector_attribute(t2);
            Eigen::Vector3d p3 = accessor.const_vector_attribute(t3);

            if (mymesh.is_ccw(t)) {
                return invariants::internal::tetrahedron_orientation(p3, p0, p1, p2);

            } else {
                return invariants::internal::tetrahedron_orientation(p3, p0, p2, p1);
            }
        }


        return true;

    } else if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
        const TriMesh& mymesh = static_cast<const TriMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 2);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            Tuple ccw_tuple = tuple;
            if (!mymesh.is_ccw(tuple)) {
                ccw_tuple = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
            }
            const wmtk::Tuple t1 = mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex);
            const wmtk::Tuple t2 =
                mymesh.switch_tuples(ccw_tuple, {PrimitiveType::Edge, PrimitiveType::Vertex});

            Eigen::Vector2d p0 = accessor.const_vector_attribute<2>(ccw_tuple);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(t1);
            Eigen::Vector3d p2 = accessor.const_vector_attribute(t2);
            if (orient2d(p0.data(), p1.data(), p2.data()) <= 0) return false;
        }


        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            const wmtk::Tuple t1 = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
            double p0 = accessor.const_scalar_attribute(tuple);
            double p1 = accessor.const_scalar_attribute(t1);

            if (p0 > p1) return false;
        }

        return true;
    }


    return true;
}


} // namespace wmtk
