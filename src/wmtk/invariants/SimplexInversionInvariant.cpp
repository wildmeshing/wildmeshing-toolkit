#include "SimplexInversionInvariant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
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
    return std::visit(
        [&](const auto& handle) noexcept -> bool {
            using T = std::decay_t<decltype(handle)>;
            constexpr static bool is_double =
                std::is_same_v<T, attribute::TypedAttributeHandle<double>>;
            constexpr static bool is_rational =
                std::is_same_v<T, attribute::utils::HybridRationalAttribute<Eigen::Dynamic>>;

            auto get_accessor = [&](const auto& mesh, auto&& dim) {
                constexpr static int D = std::decay_t<decltype(dim)>::value;
                using MeshType = std::decay_t<decltype(mesh)>;
                if constexpr (is_double) {
                    return mesh.template create_const_accessor<double, D>(handle);
                } else if constexpr (is_rational) {
                    return attribute::utils::HybridRationalAccessor<D, MeshType>(
                        const_cast<MeshType&>(mesh),
                        attribute::utils::HybridRationalAttribute<D>(handle));
                }
            };
            auto get_value = [&](const auto& acc, const Tuple& t) {
                if constexpr (is_double) {
                    return acc.const_vector_attribute(t).eval();
                } else if constexpr (is_rational) {
                    return acc.double_value(t);
                }
            };
            // static_assert(is_double || is_rational);
            if constexpr (is_double || is_rational) {
                if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron) {
                    const TetMesh& mymesh = static_cast<const TetMesh&>(mesh());
                    auto accessor = get_accessor(mymesh, std::integral_constant<int, 3>{});

                    // auto accessor = mymesh.create_const_accessor<3>(m_coordinate_handle);
                    // assert(accessor.dimension() == 3);

                    for (const auto& t : top_dimension_tuples_after) {
                        const wmtk::Tuple t1 = mymesh.switch_tuple(t, PrimitiveType::Vertex);
                        const wmtk::Tuple t2 =
                            mymesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex});
                        const wmtk::Tuple t3 = mymesh.switch_tuples(
                            t,
                            {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex});
                        const auto p0 = get_value(accessor, t);
                        const auto p1 = get_value(accessor, t1);
                        const auto p2 = get_value(accessor, t2);
                        const auto p3 = get_value(accessor, t3);

                        if (mymesh.is_ccw(t)) {
                            return invariants::internal::tetrahedron_orientation(p3, p0, p1, p2);

                        } else {
                            return invariants::internal::tetrahedron_orientation(p3, p0, p2, p1);
                        }
                    }


                    return true;

                } else if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
                    const TriMesh& mymesh = static_cast<const TriMesh&>(mesh());
                    auto accessor = get_accessor(mymesh, std::integral_constant<int, 2>{});
                    // assert(accessor.dimension() == 2);

                    for (const Tuple& tuple : top_dimension_tuples_after) {
                        Tuple ccw_tuple = tuple;
                        if (!mymesh.is_ccw(tuple)) {
                            ccw_tuple = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
                        }
                        const wmtk::Tuple t1 =
                            mymesh.switch_tuple(ccw_tuple, PrimitiveType::Vertex);
                        const wmtk::Tuple t2 = mymesh.switch_tuples(
                            ccw_tuple,
                            {PrimitiveType::Edge, PrimitiveType::Vertex});

                        const auto p0 = get_value(accessor, ccw_tuple);
                        const auto p1 = get_value(accessor, t1);
                        const auto p2 = get_value(accessor, t2);
                        return invariants::internal::triangle_orientation(p0, p1, p2);
                    }


                    return true;
                } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
                    const EdgeMesh& mymesh = static_cast<const EdgeMesh&>(mesh());
                    auto accessor = get_accessor(mymesh, std::integral_constant<int, 1>{});
                    // assert(accessor.dimension() == 1);

                    for (const Tuple& tuple : top_dimension_tuples_after) {
                        const wmtk::Tuple t1 = mymesh.switch_tuple(tuple, PrimitiveType::Vertex);
                        const auto p0 = get_value(accessor, tuple);
                        const auto p1 = get_value(accessor, t1);

                        if ((p0.array() > p1.array()).all()) return false;
                    }

                    return true;
                }
            }
            return true;
        },
        m_coordinate_handle.handle());
}


} // namespace wmtk
