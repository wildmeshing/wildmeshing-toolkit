#pragma once
#include "ConstAccessor.hpp"


namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::attribute {
template <typename T>
class MutableAccessor : public ConstAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    friend class wmtk::PointMesh;
    friend class wmtk::TriMeshOperationExecutor;
    using CachingBaseType = CachingAccessor<T>;
    using ConstAccessorType = ConstAccessor<T>;

    using ConstAccessorType::ConstAccessorType;

    using ConstAccessorType::const_scalar_attribute;
    using ConstAccessorType::const_vector_attribute;
    using ConstAccessorType::dimension;
    using ConstAccessorType::reserved_size;
    using ConstAccessorType::scalar_attribute;
    using ConstAccessorType::vector_attribute;


    using CachingBaseType::stack_depth;
    using CachingBaseType::has_stack;
protected:
    using ConstAccessorType::base_type;
    using ConstAccessorType::caching_base_type;
    using ConstAccessorType::index_access;
    using ConstAccessorType::tuple_base_type;
    CachingBaseType& index_access() { return caching_base_type(); }
};
} // namespace wmtk::attribute
