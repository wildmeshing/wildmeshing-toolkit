#pragma once

#include "TupleAccessor.hpp"

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
class TriMeshOperationExecutor;
} // namespace wmtk
namespace wmtk::attribute {


template <typename T>
class ConstAccessor : protected TupleAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    friend class wmtk::PointMesh;
    friend class wmtk::TriMeshOperationExecutor;
    using Scalar = T;

    friend class AttributeCache<T>;

    using BaseType = AccessorBase<T>;
    using TupleBaseType = TupleAccessor<T>;
    using CachingBaseType = CachingAccessor<T>;

    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>

    ConstAccessor(
        const Mesh& m,
        const MeshAttributeHandle<T>& handle,
        AttributeAccessMode access_mode = AttributeAccessMode::Immediate);


    using TupleBaseType::const_scalar_attribute;
    using TupleBaseType::const_vector_attribute;


    ConstMapResult vector_attribute(const Tuple& t) const;
    T scalar_attribute(const Tuple& t) const;

    // returns the size of the underlying attribute

    using BaseType::dimension; // const() -> long
    using BaseType::reserved_size; // const() -> long

    using BaseType::attribute; // access to Attribute object being used here
    // shows the depth of scope stacks if they exist, mostly for debug

    using CachingBaseType::stack_depth;
    using CachingBaseType::has_stack;

protected:
    using TupleBaseType::caching_base_type;
    using TupleBaseType::base_type;
    using TupleBaseType::scalar_attribute;
    using TupleBaseType::vector_attribute;


    TupleBaseType& tuple_base_type() { return *this; }
    const TupleBaseType& tuple_base_type() const { return *this; }


    const CachingBaseType& index_access() const { return caching_base_type(); }
};

} // namespace wmtk::attribute
