#pragma once
#include "AttributeTransferStrategyBase.hpp"


namespace wmtk::operations {


template <typename MyType>
class AttributeTransferStrategy : public AttributeTransferStrategyBase
{
public:
    AttributeTransferStrategy(const attribute::MeshAttributeHandle<MyType>& my_handle);
    PrimitiveType primitive_type() const override;
    Mesh& mesh() override;


    const attribute::MeshAttributeHandle<MyType>& handle() const { return m_handle; }
    attribute::MeshAttributeHandle<MyType>& handle() { return m_handle; }

    // virtual bool run(const simplex::Simplex& s)  = 0;
    bool matches_attribute(
        const wmtk::attribute::MeshAttributeHandleVariant& attr) const final override;

    // bool matches_attribute(
    //     const wmtk::attribute::MeshAttributeHandleVariant& attr,
    //     const simplex::Simplex& s) const final override;

private:
    attribute::MeshAttributeHandle<MyType> m_handle;
};


template <typename MyType, typename ParentType>
class SingleAttributeTransferStrategy : public AttributeTransferStrategy<MyType>
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
    using FunctorType = std::function<MyVecType(ParentMatType)>;

    SingleAttributeTransferStrategy(
        const attribute::MeshAttributeHandle<MyType>& my_handle,
        const attribute::MeshAttributeHandle<ParentType>& parent_handle,
        FunctorType&& = nullptr);

    void run(const simplex::Simplex& s) override;


    PrimitiveType parent_primitive_type() const;

protected:
    ParentMatType read_parent_values(const simplex::Simplex& my_simplex) const;

private:
    FunctorType m_functor;
    attribute::MeshAttributeHandle<ParentType> m_parent_handle;
};

template <typename MyType, typename ParentType>

SingleAttributeTransferStrategy(
    const MeshAttributeHandle<MyType>&,
    const MeshAttributeHandle<ParentType>&) -> SingleAttributeTransferStrategy<MyType, ParentType>;

template <typename MyType, typename ParentType, typename FunctorType>

SingleAttributeTransferStrategy(
    const MeshAttributeHandle<MyType>&,
    const MeshAttributeHandle<ParentType>&,
    FunctorType&& f) -> SingleAttributeTransferStrategy<MyType, ParentType>;

template <typename MyType, typename ParentType>
SingleAttributeTransferStrategy<MyType, ParentType>::SingleAttributeTransferStrategy(
    const MeshAttributeHandle<MyType>& me,
    const MeshAttributeHandle<ParentType>& parent,
    FunctorType&& f)
    : AttributeTransferStrategy<MyType>(me)
    , m_parent_handle(parent)
    , m_functor(f)
{}

template <typename MyType, typename ParentType>
auto SingleAttributeTransferStrategy<MyType, ParentType>::read_parent_values(
    const simplex::Simplex& my_simplex) const -> ParentMatType
{
    auto acc = m_parent_handle.create_const_accessor();
    auto simps =
        AttributeTransferStrategyBase::get_parent_simplices(handle(), m_parent_handle, my_simplex);

    MatrixX<MyType> A(m_parent_handle.dimension(), simps.size());

    using Index = Eigen::Index;
    for (Index j = 0; j < Index(simps.size()); ++j) {
        A.col(j) = acc.const_vector_attribute(simps[j]);
    }
    return A;
}
template <typename MyType, typename ParentType>
void SingleAttributeTransferStrategy<MyType, ParentType>::run(const Simplex& s)
{
    assert(mesh().is_valid_slow(s.tuple()));
    if (s.primitive_type() != primitive_type()) {
        // TODO: is this an error out or silent fail
        return;
    }

    if (m_functor) {
        auto parent_data = read_parent_values(s);
        auto acc = handle().create_accessor();

        acc.vector_attribute(s.tuple()) = m_functor(parent_data);
    }
}
template <typename MyType, typename ParentType>
PrimitiveType SingleAttributeTransferStrategy<MyType, ParentType>::parent_primitive_type() const
{
    return m_parent_handle.primitive_type();
}


} // namespace wmtk::operations
