

#pragma once
#include <vector>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk

namespace wmtk::operations::attribute_update {


template <typename MyType, typename ParentType>
class CastAttributeTransferStrategy : public SingleAttributeTransferStrategy<MyType, ParentType>
{
public:
    using BaseType = SingleAttributeTransferStrategy<MyType, ParentType>;
    using MyVecType = typename BaseType::MyVecType;
    using ParentMatType = typename BaseType::ParentMatType;

    static MyVecType convert(const ParentMatType& v)
    {
        auto eval = [](const auto& vec) -> MyVecType {
            if (vec.cols() == 1) {
                return vec;
            } else {
                if constexpr (std::is_same_v<MyType, wmtk::Rational>) {
                    return vec.rowwise().sum() / int(vec.cols());
                } else {
                    return vec.rowwise().mean();
                }
            }
        };

        if constexpr (std::is_same_v<MyType, wmtk::Rational>) {
            if constexpr (std::is_same_v<ParentType, wmtk::Rational>) {
                return eval(v);
            } else {
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
                constexpr auto cast_from_rational = [](const auto& x) -> MyType {
                    return x.to_double();
                };
                return eval(v.unaryExpr(cast_from_rational));
            } else {
                return eval(v.template cast<MyType>());
            }
        }
    }

    CastAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle)
        : BaseType(my_handle, parent_handle, &convert)
    {}
};
} // namespace wmtk::operations::utils
