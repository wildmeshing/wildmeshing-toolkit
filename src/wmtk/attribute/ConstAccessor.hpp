#pragma once

#include "TupleAccessor.hpp"

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
class TriMeshOperationExecutor;
class EdgeMesh;
namespace tests {
class DEBUG_TriMesh;
class DEBUG_EdgeMesh;
} // namespace tests
} // namespace wmtk
namespace wmtk::attribute {

/**
 * A TupleAccessor that can only read from attributes.
 */
template <typename T>
class ConstAccessor : protected TupleAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    friend class wmtk::EdgeMesh;
    friend class wmtk::PointMesh;
    friend class wmtk::TriMeshOperationExecutor;
    friend class wmtk::tests::DEBUG_TriMesh;
    friend class wmtk::tests::DEBUG_EdgeMesh;
    using Scalar = T;

    friend class AttributeCache<T>;

    using BaseType = AccessorBase<T>;
    using TupleBaseType = TupleAccessor<T>;
    using CachingBaseType = CachingAccessor<T>;

    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>

    ConstAccessor(
        const Mesh& m,
        const TypedAttributeHandle<T>& handle);
    ConstAccessor(ConstAccessor&&) = default;
    ConstAccessor& operator=(ConstAccessor&&) = default;


    using TupleBaseType::const_scalar_attribute;
    using TupleBaseType::const_vector_attribute;


    ConstMapResult vector_attribute(const Tuple& t) const;
    T scalar_attribute(const Tuple& t) const;

    // returns the size of the underlying attribute

    using BaseType::dimension; // const() -> int64_t
    using BaseType::reserved_size; // const() -> int64_t

    using BaseType::attribute; // access to Attribute object being used here
    // shows the depth of scope stacks if they exist, mostly for debug

    using CachingBaseType::has_stack;
    using CachingBaseType::stack_depth;

protected:
    using TupleBaseType::base_type;
    using TupleBaseType::caching_base_type;
    using TupleBaseType::scalar_attribute;
    using TupleBaseType::vector_attribute;


    TupleBaseType& tuple_base_type() { return *this; }
    const TupleBaseType& tuple_base_type() const { return *this; }


    const CachingBaseType& index_access() const { return caching_base_type(); }
};

/*
// This implementation lies here to avoid dragging too many definitions
// (Some code doesn't require accessors and therefore don't include them)
// header is in MeshAttributeHandle.hpp
template <typename T>
ConstAccessor<T> MeshAttributeHandle<T>::create_const_accessor() const
{
    return mesh().create_const_accessor(*this);
}
template <typename T>
ConstAccessor<T> MeshAttributeHandle<T>::create_accessor() const
{
    return create_const_accessor();
}
*/

} // namespace wmtk::attribute
