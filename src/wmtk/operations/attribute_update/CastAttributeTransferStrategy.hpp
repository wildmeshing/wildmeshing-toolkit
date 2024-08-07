

#pragma once
#include <vector>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk

namespace wmtk::operations::utils {


template <typename MyType, typename ParentType>
class CastAttributeTransferStrategy : public SingleAttributeTransferStrategy<MyType, ParentType>
{
public:
    using BaseType = SingleAttributeTransferStrategy<MyType, ParentType>;
    using MyVecType = typename BaseType::MyVecType;
    using ParentMatType = typename BaseType::ParentMatType;

    static MyVecType convert(const ParentMatType& v)
    {
        constexpr static bool is_hybrid_rational = std::is_same_v<
            ParentType,
            wmtk::attribute::utils::HybridRationalAttribute<Eigen::Dynamic>::Type>;
        if constexpr (is_hybrid_rational) {
            assert(false);
            return {};
        } else if constexpr (
            std::is_same_v<MyType, wmtk::Rational> && !std::is_same_v<ParentType, wmtk::Rational>) {
            constexpr auto cast_rational = [](const auto& x) -> MyType {
                if constexpr (std::is_same_v<ParentType, double>) {
                    return wmtk::Rational(x, true);
                } else {
                    return wmtk::Rational(int(x), true);
                }
            };
            if (v.cols() == 1) {
                return v.unaryExpr(cast_rational);
            } else {
                return v.unaryExpr(cast_rational).rowwise().sum() / wmtk::Rational(int(v.cols()));
            }


        } else {
            if (v.cols() == 1) {
                return v.template cast<MyType>();
            } else {
                if constexpr (std::is_same_v<MyType, wmtk::Rational>) {
                    return v.template cast<MyType>().rowwise().sum() / int(v.cols());
                } else {
                    return v.template cast<MyType>().rowwise().mean();
                }
            }
            return {};
        }
    }

    CastAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle)
        : BaseType(my_handle, parent_handle, &convert)
    {}
};
} // namespace wmtk::operations::utils
