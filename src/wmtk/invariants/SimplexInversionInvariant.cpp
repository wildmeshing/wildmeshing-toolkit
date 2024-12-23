#include "SimplexInversionInvariant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk {

template <typename T>
SimplexInversionInvariant<T>::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<T>& coordinate,
    bool inverted)
    : Invariant(m, true, false, true)
    , m_coordinate_handle(coordinate)
    , m_inverted(inverted)
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
            const auto tet_vertices = mymesh.orient_vertices(t);
            assert(tet_vertices.size() == 4);
            const Eigen::Vector3<T> p0 = accessor.const_vector_attribute(tet_vertices[0]);
            const Eigen::Vector3<T> p1 = accessor.const_vector_attribute(tet_vertices[1]);
            const Eigen::Vector3<T> p2 = accessor.const_vector_attribute(tet_vertices[2]);
            const Eigen::Vector3<T> p3 = accessor.const_vector_attribute(tet_vertices[3]);

            if (utils::wmtk_orient3d(p0, p1, p2, p3) <= 0) {
                wmtk::logger().debug(
                    "fail inversion check, volume = {}",
                    utils::wmtk_orient3d(p0, p1, p2, p3));
                return false;
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

            if (!is_oriented(p0, p1, p2)) return false;
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

            if (!is_oriented(p0, p1)) return false;
        }

        return true;
    }

    return true;
}

template <typename T>
bool SimplexInversionInvariant<T>::is_oriented(
    const Eigen::Ref<const Vector1<T>>& p0,
    const Eigen::Ref<const Vector1<T>>& p1) const
{
    return is_oriented(p0.x(), p1.x());
}
template <typename T>
bool SimplexInversionInvariant<T>::is_oriented(const T& p0, const T& p1) const
{
    //
    const int orient = utils::wmtk_orient1d(p0, p1);
    if (m_inverted) {
        return orient < 0;
    } else {
        return orient > 0;
    }
}
template <typename T>
bool SimplexInversionInvariant<T>::is_oriented(
    const Eigen::Ref<const Vector2<T>>& p0,
    const Eigen::Ref<const Vector2<T>>& p1,
    const Eigen::Ref<const Vector2<T>>& p2) const
{
    //
    const int orient = utils::wmtk_orient2d(p0, p1, p2);
    if (m_inverted) {
        return orient < 0;
    } else {
        return orient > 0;
    }
}
template <typename T>
bool SimplexInversionInvariant<T>::is_oriented(
    const Eigen::Ref<const Vector3<T>>& p0,
    const Eigen::Ref<const Vector3<T>>& p1,
    const Eigen::Ref<const Vector3<T>>& p2,
    const Eigen::Ref<const Vector3<T>>& p3) const
{
    const int orient = utils::wmtk_orient3d(p0, p1, p2, p3);
    if (m_inverted) {
        return orient < 0;
    } else {
        return orient > 0;
    }
    //
}

template class SimplexInversionInvariant<double>;
template class SimplexInversionInvariant<Rational>;

} // namespace wmtk
