#pragma once
#include "ConstAccessor.hpp"


namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::attribute {

/**
 * Same as ConstAccessor but with the ability to write to the attributes.
 */
template <typename T>
class MutableAccessor : public ConstAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    friend class wmtk::EdgeMesh;
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


    using CachingBaseType::has_stack;
    using CachingBaseType::mesh;
    using CachingBaseType::stack_depth;

    using ConstAccessorType::mesh;

protected:
    using ConstAccessorType::base_type;
    using ConstAccessorType::caching_base_type;
    using ConstAccessorType::index_access;
    using ConstAccessorType::tuple_base_type;
    CachingBaseType& index_access() { return caching_base_type(); }
};


/*
// This implementation lies here to avoid dragging too many definitions
// (Some code doesn't require accessors and therefore don't include them)
// header is in MeshAttributeHandle.hpp
template <typename T>
MutableAccessor<T> MeshAttributeHandle<T>::create_accessor()
{
    return mesh().create_accessor(*this);
}
*/
} // namespace wmtk::attribute
