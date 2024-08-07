

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
        if constexpr (
            std::is_same_v<MyType, wmtk::Rational> && !std::is_same_v<ParentType, wmtk::Rational>) {
            return v.unaryExpr([](const auto& x) -> MyType { return wmtk::Rational(x, true); });


        } else {
            return v.template cast<MyType>();
        }
    }

    CastAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle)
        : BaseType(my_handle, parent_handle, &convert)
    {}
};
} // namespace wmtk::operations::utils
