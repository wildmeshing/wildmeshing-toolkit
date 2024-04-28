#include "SimplexInversionInvariant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/orient.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk {

template <typename T>
SimplexInversionInvariant<T>::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<T>& coordinate)
    : Invariant(m, true, false, true)
    , m_coordinate_handle(coordinate)
{}

template <typename T>
bool SimplexInversionInvariant<T>::after(
    const std::vector<Tuple>&,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
        const auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            const Eigen::Vector3<T> p0 = accessor.const_vector_attribute(t);
            const Eigen::Vector3<T> p1 =
                accessor.const_vector_attribute(mymesh.switch_tuple(t, PrimitiveType::Vertex));
            const Eigen::Vector3<T> p2 = accessor.const_vector_attribute(
                mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
            const Eigen::Vector3<T> p3 = accessor.const_vector_attribute(mymesh.switch_tuples(
                t,
                {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));

            if (mymesh.is_ccw(t)) {
                if (utils::wmtk_orient3d<T>(p3, p0, p1, p2) <= 0) return false;
            } else {
                if (utils::wmtk_orient3d<T>(p3, p0, p2, p1) <= 0) return false;
            }
        }

        return true;

    } else if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
        const TriMesh& mymesh = static_cast<const TriMesh&>(mesh());
        const auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 2);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            Tuple ccw_tuple = tuple;
            if (!mymesh.is_ccw(tuple)) {
                ccw_tuple = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
            }
            const Eigen::Vector2<T> p0 = accessor.const_vector_attribute(ccw_tuple);
            const Eigen::Vector2<T> p1 = accessor.const_vector_attribute(
                mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex));
            const Eigen::Vector2<T> p2 = accessor.const_vector_attribute(
                mymesh.switch_tuples(ccw_tuple, {PrimitiveType::Edge, PrimitiveType::Vertex}));

            if (utils::wmtk_orient2d<T>(p0, p1, p2) <= 0) return false;
        }

        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
        const auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            T p0 = accessor.const_scalar_attribute(tuple);
            T p1 =
                accessor.const_scalar_attribute(mymesh.switch_tuple(tuple, PrimitiveType::Vertex));

            if (utils::wmtk_orient1d(p0, 01) >= 0) return false;
        }

        return true;
    }

    return true;
}

template class SimplexInversionInvariant<double>;
template class SimplexInversionInvariant<Rational>;

} // namespace wmtk
