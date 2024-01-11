#pragma once
#include "AttributeTransferStrategy.hpp"

namespace wmtk::operations {
template <typename MyType, typename ParentType>
class FunctionAttributeTransferStrategy : public AttributeTransferStrategy<MyType>
{
public:
    using AttributeTransferStrategy<MyType>::handle;
    using AttributeTransferStrategy<MyType>::primitive_type;
    using AttributeTransferStrategy<MyType>::mesh;
    using AttributeTransferStrategy<MyType>::matches_attribute;

    template <typename T>
    using VecType = VectorX<T>;
    template <typename T>
    using MatType = MatrixX<T>;
    using MyVecType = VecType<MyType>;
    using ParentMatType = MatType<ParentType>;


    // you can pass as many COLUMN vectors as you want to the function depending on the relative
    // locations of simplices
    // if the simplex for update() is uniquely represented after a lub_map then the order of
    // simplices received is guaranteed to follow that of faces_single_dimension or
    // cofaces_single_dimension. Otherwise data is recieved in an arbitrary order.
    using FunctorWithoutSimplicesType = std::function<MyVecType(const ParentMatType&)>;
    using FunctorType = std::function<MyVecType(const ParentMatType&, const std::vector<Tuple>&)>;

    SingleAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle,
        FunctorType&& = nullptr);
    SingleAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle,
        FunctorWithoutSimplicesType&& = nullptr);

    void run(const simplex::Simplex& s) override;


    PrimitiveType parent_primitive_type() const;

protected:
    std::pair<ParentMatType, std::vector<Tuple>> read_parent_values(
        const simplex::Simplex& my_simplex) const;

private:
    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction>
    attribute::MeshAttributeHandle m_parent_handle;
};

} // namespace wmtk::operations
