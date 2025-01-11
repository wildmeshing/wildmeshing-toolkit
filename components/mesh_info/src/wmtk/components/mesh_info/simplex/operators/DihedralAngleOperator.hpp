#p
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
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
    : public wmtk::operations::AttributeTransferStrategy<ParentType>
{
public:
    using BaseType =
        wmtk::operations::AttributeTransferStrategy<ParentType>;
    using MyVecType = typename BaseType::MyVecType;
    using ParentMatType = typename BaseType::ParentMatType;

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

    DihedralAngleOperator(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle)
        : BaseType(my_handle, parent_handle, &convert)
    {
        assert(parent_handle.primitive_type() == PrimitiveType::Vertex);
    }
private:
    attribute::MeshAttributeHandle m_parent_handle;
};
} // namespace wmtk::components::mesh_info::simplex::operators
