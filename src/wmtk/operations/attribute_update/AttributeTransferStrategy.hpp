#pragma once
#include <wmtk/Mesh.hpp>
#include "AttributeTransferStrategyBase.hpp"


namespace wmtk::operations {


template <typename MyType>
class AttributeTransferStrategy : public AttributeTransferStrategyBase
{
public:
    AttributeTransferStrategy(const attribute::MeshAttributeHandle& my_handle);
    PrimitiveType primitive_type() const override;
    Mesh& mesh() override;
    using AttributeTransferStrategyBase::mesh;


    // bool matches_attribute(
    //     const wmtk::attribute::MeshAttributeHandle& attr,
    //     const simplex::Simplex& s) const final;
};


template <
    typename MyType,
    typename ParentType>
class SingleAttributeTransferStrategyBase : public AttributeTransferStrategy<MyType>
{
public:
    using AttributeTransferStrategy<MyType>::handle;
    using AttributeTransferStrategy<MyType>::primitive_type;
    using AttributeTransferStrategy<MyType>::mesh;
    using AttributeTransferStrategy<MyType>::matches_attribute;

    SingleAttributeTransferStrategyBase(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& parent_handle);


    PrimitiveType parent_primitive_type() const;

    std::vector<wmtk::attribute::MeshAttributeHandle> sources() const final
    {
        return {m_parent_handle};
    }


    const attribute::MeshAttributeHandle& parent_handle() const { return  m_parent_handle;}
protected:
    attribute::MeshAttributeHandle m_parent_handle;
};
/**
 * @tparam MyType The type to which transfer should go to.
 * @tparam ParentType The type that causes the change in MyType.
 */
template <
    typename MyType,
    typename ParentType,
    int MyDim = Eigen::Dynamic,
    int ParentDim = Eigen::Dynamic>
class SingleAttributeTransferStrategy : public SingleAttributeTransferStrategyBase<MyType, ParentType>
{
public:
    using AttributeTransferStrategy<MyType>::handle;
    using SingleAttributeTransferStrategy<MyType,ParentType>::parent_handle;
    using AttributeTransferStrategy<MyType>::primitive_type;
    using AttributeTransferStrategy<MyType>::mesh;
    using AttributeTransferStrategy<MyType>::matches_attribute;

    using MyVecType = Vector<MyType, MyDim>;
    using ParentMatType = ColVectors<ParentType, MyDim>;


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

    void run(const simplex::Simplex& s) const final;


    static FunctorType make_nosimplices_func(FunctorWithoutSimplicesType&& fp)
    {
        return
            [fp](const ParentMatType& a, const std::vector<Tuple>&) -> MyVecType { return fp(a); };
    }


protected:
    std::pair<ParentMatType, std::vector<Tuple>> read_parent_values(
        const simplex::Simplex& my_simplex) const;

private:
    FunctorType m_functor;
};

// template <typename MyType, typename ParentType>

// SingleAttributeTransferStrategy(
//     const attribute::MeshAttributeHandle&,
//     const attribute::MeshAttributeHandle&) -> SingleAttributeTransferStrategy<MyType,
//     ParentType>;

// template <typename MyType, typename ParentType, typename FunctorType>

// SingleAttributeTransferStrategy(
//     const attribute::MeshAttributeHandle&,
//     const attribute::MeshAttributeHandle&,
//     FunctorType&& f) -> SingleAttributeTransferStrategy<MyType, ParentType>;

template <typename MyType, typename ParentType>
SingleAttributeTransferStrategyBase<MyType, ParentType>::
    SingleAttributeTransferStrategyBase(
        const attribute::MeshAttributeHandle& me,
        const attribute::MeshAttributeHandle& parent)
    : AttributeTransferStrategy<MyType>(me)
    , m_parent_handle(parent)
{
    assert(me.template holds<MyType>());
    assert(parent.template holds<ParentType>());
}

template <typename MyType, typename ParentType, int MyDim, int ParentDim>
SingleAttributeTransferStrategy<MyType, ParentType, MyDim, ParentDim>::
    SingleAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& me,
        const attribute::MeshAttributeHandle& parent,
        FunctorType&& f)
    : SingleAttributeTransferStrategyBase<MyType,ParentType>(me,parent)
    , m_functor(f)
{
}
template <typename MyType, typename ParentType, int MyDim, int ParentDim>
SingleAttributeTransferStrategy<MyType, ParentType, MyDim, ParentDim>::
    SingleAttributeTransferStrategy(
        const attribute::MeshAttributeHandle& me,
        const attribute::MeshAttributeHandle& parent,
        FunctorWithoutSimplicesType&& f)
    : SingleAttributeTransferStrategy(me, parent, make_nosimplices_func(std::move(f)))
{}

template <typename MyType, typename ParentType, int MyDim, int ParentDim>
auto SingleAttributeTransferStrategy<MyType, ParentType, MyDim, ParentDim>::read_parent_values(
    const simplex::Simplex& my_simplex) const -> std::pair<ParentMatType, std::vector<Tuple>>
{
    auto acc =
        parent_handle().mesh().create_const_accessor(parent_handle().template as<ParentType>());
    auto simps =
        AttributeTransferStrategyBase::get_parent_simplices(handle(), parent_handle(), my_simplex);

    MatrixX<ParentType> A(
        parent_handle().mesh().get_attribute_dimension(parent_handle().template as<ParentType>()),
        simps.size());

    using Index = Eigen::Index;
    for (Index j = 0; j < Index(simps.size()); ++j) {
        A.col(j) = acc.template const_vector_attribute<ParentDim>(simps[j]);
    }
    return std::make_pair(std::move(A), std::move(simps));
}
template <typename MyType, typename ParentType, int MyDim, int ParentDim>
void SingleAttributeTransferStrategy<MyType, ParentType, MyDim, ParentDim>::run(
    const simplex::Simplex& s) const
{
    assert(mesh().is_valid(s.tuple()));
    if (s.primitive_type() != primitive_type()) {
        // TODO: is this an error out or silent fail
        return;
    }

    if (m_functor) {
        auto [parent_data, simps] = read_parent_values(s);
        if (simps.empty()) return;
        auto acc = const_cast<Mesh&>(mesh()).create_accessor(handle().template as<MyType>());

        acc.template vector_attribute<MyDim>(s.tuple()) = m_functor(parent_data, simps);
    }
}
template <typename MyType, typename ParentType>
PrimitiveType
SingleAttributeTransferStrategyBase<MyType, ParentType>::parent_primitive_type() const
{
    return m_parent_handle.primitive_type();
}

} // namespace wmtk::operations
