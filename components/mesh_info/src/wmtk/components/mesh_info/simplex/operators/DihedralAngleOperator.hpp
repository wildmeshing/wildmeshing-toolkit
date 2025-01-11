#p
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/triangle_areas.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk
namespace wmtk::components::mesh_info::simplex::operators {

template <typename MyType, typename ParentType, int ParentDim = Eigen::Dynamic>
class DihedralAngleOperator
    : public wmtk::operations::SingleAttributeTransferStrategyBase<MyType, ParentType>
{
public:
    using BaseType = wmtk::operations::SingleAttributeTransferStrategyBase<MyType, ParentType>;

    using BaseType::handle;
    using BaseType::mesh;
    using BaseType::parent_handle;
    using BaseType::primitive_type;
    /*
    static MyVecType convert(const ParentMatType& v)
    {
        constexpr static std::array<int, 4> quot = {{1, 1, 2, 6}};
        auto eval = [](const auto& vec) -> MyVecType {
            using T = typename MyVecType::Scalar;
            if constexpr (
                std::is_same_v<T, int64_t> || std::is_same_v<T, char> ||
                std::is_same_v<T, wmtk::Rational>) {
                assert(false);
                return MyVecType::Constant(T(1));
            } else {
                auto m = (vec.rightCols(vec.cols() - 1).colwise() - vec.col(0)).eval();
                int quotient = quot[m.cols()];
                if (vec.cols() == vec.rows() + 1) {
                    return MyVecType::Constant(m.determinant() / T(quotient));
                } else {
                    return MyVecType::Constant(
                        std::sqrt((m.transpose() * m).determinant()) / T(quotient));
                }
            }
        };

        if constexpr (std::is_same_v<MyType, wmtk::Rational>) {
            if constexpr (std::is_same_v<ParentType, wmtk::Rational>) {
                // rational -> rational
                return eval(v);
            } else {
                // sometype -> rational
                constexpr auto cast_rational = [](const auto& x) -> MyType {
                    if constexpr (std::is_same_v<ParentType, double>) {
                        return wmtk::Rational(x, true);
                    } else {
                        return wmtk::Rational(int(x), true);
                    }
                };
                return eval(v.unaryExpr(cast_rational));
            }


        } else { // my type is not rational
            if constexpr (std::is_same_v<ParentType, wmtk::Rational>) {
                // rational -> type
                constexpr auto cast_from_rational = [](const auto& x) -> MyType {
                    return x.to_double();
                };
                return eval(v.unaryExpr(cast_from_rational));
            } else {
                // type -> type
                return eval(v.template cast<MyType>());
            }
        }
    }
    */

    void run(const wmtk::simplex::Simplex& s) const final
    {
        assert(handle().mesh().is_valid(s.tuple()));
        if (s.primitive_type() != primitive_type()) {
            // TODO: is this an error out or silent fail
            return;
        }
        // position mesh
        const TriMesh& pmesh = static_cast<const TriMesh&>(parent_handle().mesh());

        auto ang_acc =
            parent_handle().mesh().create_const_accessor(parent_handle().template as<ParentType>());
        auto acc = const_cast<Mesh&>(mesh()).create_accessor(handle().template as<MyType>());

        auto& angle = acc.scalar_attribute(s.tuple());
        if (pmesh.is_boundary(s)) {
            angle = 0;
        } else {
            auto pos_acc = parent_handle().mesh().template create_const_accessor<double, 3>(
                parent_handle().template as<ParentType>());


            // edge a -> b; input triangle has edge c, opp has d
            //      /c
            //     / |
            //    /t |
            //   /   |
            //   a---b
            //   \   |
            //    \  |
            //     \ |
            //      \d
            auto a = pos_acc.const_vector_attribute(s.tuple());

            auto b = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Vertex}));

            auto c = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Edge}));
            auto d = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Triangle, PrimitiveType::Edge}));

            auto ban = (b - a).normalized().eval();
            auto ca = (c - a).eval();
            auto da = (d - a).eval();
            ca.noalias() -= ca.dot(ban) * ban;
            da.noalias() -= da.dot(ban) * ban;

            return std::atan2(ca.cross(da).norm(), ca.dot(da));
        }
    }

    DihedralAngleOperator(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle)
        : BaseType(my_handle, parent_handle)
    {
        assert(parent_handle.mesh().top_simplex_type() == PrimitiveType::Triangle);
        assert(parent_handle.primitive_type() == PrimitiveType::Vertex);
        assert(parent_handle.dimension() == 3);
        assert(my_handle.primitive_type() == PrimitiveType::Edge);
    }

private:
    attribute::MeshAttributeHandle m_parent_handle;
};
} // namespace wmtk::components::mesh_info::simplex::operators
