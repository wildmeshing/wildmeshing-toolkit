#include "SimplexInversionInvariant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "predicates.h"

namespace wmtk {

namespace {
bool is_rounded(const Eigen::Vector3<Rational>& p)
{
    return p[0].is_rounded() && p[1].is_rounded() && p[2].is_rounded();
}
bool is_rounded(const Eigen::Vector2<Rational>& p)
{
    return p[0].is_rounded() && p[1].is_rounded();
}
} // namespace

SimplexInversionInvariant<double>::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate)
    : Invariant(m, true, false, true)
    , m_coordinate_handle(coordinate)
{}

bool SimplexInversionInvariant<double>::after(
    const std::vector<Tuple>&,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute<3>(t);
            Eigen::Vector3d p1 =
                accessor.const_vector_attribute<3>(mymesh.switch_tuple(t, PrimitiveType::Vertex));
            Eigen::Vector3d p2 = accessor.const_vector_attribute<3>(
                mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
            Eigen::Vector3d p3 = accessor.const_vector_attribute<3>(mymesh.switch_tuples(
                t,
                {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));

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
                ccw_tuple = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
            }
            Eigen::Vector2d p0 = accessor.const_vector_attribute<2>(ccw_tuple);
            Eigen::Vector2d p1 = accessor.const_vector_attribute<2>(
                mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex));
            Eigen::Vector2d p2 = accessor.const_vector_attribute<2>(
                mymesh.switch_tuples(ccw_tuple, {PrimitiveType::Edge, PrimitiveType::Vertex}));

            if (orient2d(p0.data(), p1.data(), p2.data()) <= 0) return false;
        }


        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            double p0 = accessor.const_scalar_attribute(tuple);
            double p1 =
                accessor.const_scalar_attribute(mymesh.switch_tuple(tuple, PrimitiveType::Vertex));

            if (p0 > p1) return false;
        }

        return true;
    }


    return true;
}

//////////////////////////// Rational

SimplexInversionInvariant<Rational>::SimplexInversionInvariant(
    const Mesh& m,
    const TypedAttributeHandle<Rational>& coordinate)
    : Invariant(m, true, false, true)
    , m_coordinate_handle(coordinate)
{}

bool SimplexInversionInvariant<Rational>::after(
    const std::vector<Tuple>&,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
        const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 3);

        for (const auto& t : top_dimension_tuples_after) {
            const Eigen::Vector3<Rational> p0 = accessor.const_vector_attribute<3>(t);
            const Eigen::Vector3<Rational> p1 =
                accessor.const_vector_attribute<3>(mymesh.switch_tuple(t, PrimitiveType::Vertex));
            const Eigen::Vector3<Rational> p2 = accessor.const_vector_attribute<3>(
                mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
            const Eigen::Vector3<Rational> p3 =
                accessor.const_vector_attribute<3>(mymesh.switch_tuples(
                    t,
                    {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));

            if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2) && is_rounded(p3)) {
                Eigen::Vector3d p0r = p0.cast<double>();
                Eigen::Vector3d p1r = p1.cast<double>();
                Eigen::Vector3d p2r = p2.cast<double>();
                Eigen::Vector3d p3r = p3.cast<double>();
                if (mymesh.is_ccw(t)) {
                    if (orient3d(p3r.data(), p0r.data(), p1r.data(), p2r.data()) <= 0) return false;

                } else {
                    if (orient3d(p3r.data(), p0r.data(), p2r.data(), p1r.data()) <= 0) return false;
                }
            } else {
                Eigen::Matrix3<Rational> M;
                M.row(0) = p1 - p0;
                M.row(1) = p2 - p0;
                M.row(2) = p3 - p0;
                const auto det = M.determinant();

                if (mymesh.is_ccw(t)) {
                    if (det >= 0) return false;

                } else {
                    if (det <= 0) return false;
                }
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
            Eigen::Vector2<Rational> p0 = accessor.const_vector_attribute<2>(ccw_tuple);
            Eigen::Vector2<Rational> p1 = accessor.const_vector_attribute<2>(
                mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex));
            Eigen::Vector2<Rational> p2 = accessor.const_vector_attribute<2>(
                mymesh.switch_tuples(ccw_tuple, {PrimitiveType::Edge, PrimitiveType::Vertex}));
            if (is_rounded(p0) && is_rounded(p1) && is_rounded(p2)) {
                Eigen::Vector2d p0r = p0.cast<double>();
                Eigen::Vector2d p1r = p1.cast<double>();
                Eigen::Vector2d p2r = p2.cast<double>();

                if (orient2d(p0r.data(), p1r.data(), p2r.data()) <= 0) return false;
            } else {
                Eigen::Matrix2<Rational> M;
                M.row(0) = p1 - p0;
                M.row(1) = p2 - p0;
                return M.determinant() >= 0;
            }
        }


        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
        auto accessor = mymesh.create_const_accessor(m_coordinate_handle);
        assert(accessor.dimension() == 1);

        for (const Tuple& tuple : top_dimension_tuples_after) {
            Rational p0 = accessor.const_scalar_attribute(tuple);
            Rational p1 =
                accessor.const_scalar_attribute(mymesh.switch_tuple(tuple, PrimitiveType::Vertex));

            if (p0 > p1) return false;
        }

        return true;
    }


    return true;
}


} // namespace wmtk
