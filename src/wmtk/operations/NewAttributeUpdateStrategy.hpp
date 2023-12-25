#pragma once
#include <variant>
#include <wmtk/Rational.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
// #include "NewAttributeStrategy.hpp"


namespace wmtk::operations {


class NewAttributeUpdateStrategyBase
{
    // placeholder for when this turns into a DAG that needs to be linearized
    using HandleVariant = std::variant<
        attribute::MeshAttributeHandle<double>,
        attribute::MeshAttributeHandle<long>,
        attribute::MeshAttributeHandle<char>,
        attribute::MeshAttributeHandle<Rational>>;

    // if you map a j-simplex to a k-simplex then
    // if j < k: the system evaluates the function at every j-simplex that is a face of k
    // if k < j: the system evaluates the function at every j-simplex that is a coface of k
    // if j == k: the system assumes a point-wise update with no neighbors
    //
    static std::vector<Tuple>
    get_parent_simplices(const Mesh& m, const Simplex& s, PrimitiveType pt);

public:
    // placeholder for when this turns into a DAG that needs to be linearized
    // virtual std::vector<HandleVariant> parent_handles() const = 0;

    virtual PrimitiveType primitive_type() const = 0;
};

template <typename MyType>
class NewAttributeUpdateStrategy
{
public:
    NewAttributeUpdateStrategy(const attribute::MeshAttributeHandle<MyType>& my_handle);
    PrimitiveType primitive_type() const override;

private:
    attribute::MeshAttributeHandle<T> m_handle;
};


template <typename MyType, typename ParentType>
class SingleNewAttributeUpdateStrategy : public NewAttributeUpdateStrategy
{
public:
    NewAttributeUpdateStrategy(
        const attribute::MeshAttributeHandle<MyType>& my_handle,
        const std::MeshAttributeHandle<ParentType>& parent_handle);

    template <typename T>
    using VecType = VectorX<T>;
    template <typename T>
    using MatType = MatrixX<T>;
    using MyVecType = typename VecType<MyType>;
    using ParentMatType = typenameMatType<ParentType>;

    // you can pass as many COLUMN vectors as you want to the function depending on the relative
    // locations of simplices
    using FunctorType = std::function<MyType(std::vector<ParentVecType>)>;

    PrimitiveType parent_primitive_type() const;

protected:
    ParentMatType read_parent_values() const;

private:
    FunctorType m_functor;
    attribute::MeshAttributeHandle<ParentType> m_parent_handle;
};

} // namespace wmtk::operations
