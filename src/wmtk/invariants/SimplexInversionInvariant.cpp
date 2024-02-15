#include "SimplexInversionInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "predicates.h"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/TetMesh.hpp>

namespace wmtk {
SimplexInversionInvariant::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate)
    : Invariant(m, true, false, true)
    , m_coordinate_handle(coordinate)
{}

bool SimplexInversionInvariant::after(
    const std::vector<Tuple>&,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{

    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute<3>(t);
            Eigen::Vector3d p1 = accessor.const_vector_attribute<3>(mymesh.switch_tuple(t, PrimitiveType::Vertex));
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute<3>(mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
            Eigen::Vector3d p3 = accessor.const_vector_attribute<3>(
                mymesh.switch_tuples(t,{PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));

            if (mymesh.is_ccw(t)) {
                if (orient3d(p3.data(), p0.data(), p1.data(), p2.data()) <= 0) return false;

            } else {
                if (orient3d(p3.data(), p0.data(), p2.data(), p1.data()) <= 0) return false;
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
                ccw_tuple = mymesh.switch_tuple(tuple,PrimitiveType::Vertex);
            }
            Eigen::Vector2d p0 = accessor.const_vector_attribute<2>(ccw_tuple);
            Eigen::Vector2d p1 = accessor.const_vector_attribute<2>(mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex));
            Eigen::Vector2d p2 = accessor.const_vector_attribute<2>(
                    mymesh.switch_tuples(ccw_tuple, {PrimitiveType::Edge,  PrimitiveType::Vertex}));

            if (orient2d(p0.data(), p1.data(), p2.data()) <= 0) return false;
        }


        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            double p0 = accessor.const_scalar_attribute(tuple);
            double p1 = accessor.const_scalar_attribute(mymesh.switch_tuple(tuple, PrimitiveType::Vertex));

            if (p0 > p1) return false;
        }

        return true;
    }


    return true;
}


} // namespace wmtk
