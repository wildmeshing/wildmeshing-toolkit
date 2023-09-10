#pragma once
#include "CachingAccessor.hpp"

namespace wmtk {
class Mesh;
class TetMesh;
class TriMesh;
} // namespace wmtk
namespace wmtk::attribute {
template <typename T>
class TupleAccessor : protected CachingAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    using Scalar = T;

    friend class AttributeCache<T>;
    using BaseType = AccessorBase<T>;
    using CachingBaseType = CachingAccessor<T>;

    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>
    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>

    using CachingBaseType::CachingBaseType;

    TupleAccessor(const TupleAccessor&) = delete;
    TupleAccessor& operator=(const TupleAccessor&) = delete;


    T const_scalar_attribute(const Tuple& t) const;
    T& scalar_attribute(const Tuple& t);

    ConstMapResult const_vector_attribute(const Tuple& t) const;
    MapResult vector_attribute(const Tuple& t);

    long index(const Tuple& t) const;
    using BaseType::dimension; // const() -> long
    using BaseType::reserved_size; // const() -> long

    using BaseType::attribute; // access to Attribute object being used here
    using CachingBaseType::mesh;
    using CachingBaseType::stack_depth;
    using CachingBaseType::has_stack;
protected:
    using CachingBaseType::base_type;
    CachingBaseType& caching_base_type() { return *this; }
    const CachingBaseType& caching_base_type() const { return *this; }
};
} // namespace wmtk::attribute
