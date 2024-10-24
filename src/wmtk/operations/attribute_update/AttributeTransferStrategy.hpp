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
    //     const simplex::Simplex& s) const final override;
};

/**
 * @tparam MyType The type to which transfer should go to.
 * @tparam ParentType The type that causes the change in MyType.
 */
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

    void run(const simplex::Simplex& s) const final override;


    PrimitiveType parent_primitive_type() const;

    static FunctorType make_nosimplices_func(FunctorWithoutSimplicesType&& fp)
    {
        return
            [fp](const ParentMatType& a, const std::vector<Tuple>&) -> MyVecType { return fp(a); };
    }
    std::vector<wmtk::attribute::MeshAttributeHandle> sources() const final override
    {
        return {m_parent_handle};
    }


protected:
    std::pair<ParentMatType, std::vector<Tuple>> read_parent_values(
        const simplex::Simplex& my_simplex) const;

private:
    FunctorType m_functor;
    attribute::MeshAttributeHandle m_parent_handle;
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
SingleAttributeTransferStrategy<MyType, ParentType>::SingleAttributeTransferStrategy(
    const attribute::MeshAttributeHandle& me,
    const attribute::MeshAttributeHandle& parent,
    FunctorType&& f)
    : AttributeTransferStrategy<MyType>(me)
    , m_functor(f)
    , m_parent_handle(parent)
{
    assert(me.template holds<MyType>());
    assert(parent.template holds<ParentType>());
}
template <typename MyType, typename ParentType>
SingleAttributeTransferStrategy<MyType, ParentType>::SingleAttributeTransferStrategy(
    const attribute::MeshAttributeHandle& me,
    const attribute::MeshAttributeHandle& parent,
    FunctorWithoutSimplicesType&& f)
    : SingleAttributeTransferStrategy(me, parent, make_nosimplices_func(std::move(f)))
{}

template <typename MyType, typename ParentType>
auto SingleAttributeTransferStrategy<MyType, ParentType>::read_parent_values(
    const simplex::Simplex& my_simplex) const -> std::pair<ParentMatType, std::vector<Tuple>>
{
    auto acc =
        m_parent_handle.mesh().create_const_accessor(m_parent_handle.template as<ParentType>());
    auto simps =
        AttributeTransferStrategyBase::get_parent_simplices(handle(), m_parent_handle, my_simplex);

    MatrixX<ParentType> A(
        m_parent_handle.mesh().get_attribute_dimension(m_parent_handle.template as<ParentType>()),
        simps.size());

    using Index = Eigen::Index;
    for (Index j = 0; j < Index(simps.size()); ++j) {
        A.col(j) = acc.const_vector_attribute(simps[j]);
    }
    return std::make_pair(std::move(A), std::move(simps));
}
template <typename MyType, typename ParentType>
void SingleAttributeTransferStrategy<MyType, ParentType>::run(const simplex::Simplex& s) const
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

        acc.vector_attribute(s.tuple()) = m_functor(parent_data, simps);
    }
}
template <typename MyType, typename ParentType>
PrimitiveType SingleAttributeTransferStrategy<MyType, ParentType>::parent_primitive_type() const
{
    return m_parent_handle.primitive_type();
}

} // namespace wmtk::operations
