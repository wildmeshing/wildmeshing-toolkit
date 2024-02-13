#pragma once
#include "CachingAccessor.hpp"

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
 * A CachingAccessor that uses tuples for accessing attributes instead of indices.
 * As global simplex ids should not be publicly available, this accessor uses the Mesh.id() function
 * to map from a tuple to the global simplex id.
 */
template <typename T>
class Accessor : protected CachingAccessor<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    friend class wmtk::EdgeMesh;
    friend class wmtk::PointMesh;
    using Scalar = T;

    friend class AttributeCache<T>;
    using BaseType = AccessorBase<T>;
    using CachingBaseType = CachingAccessor<T>;

    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>
    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>

    using CachingBaseType::CachingBaseType;


    T const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt) const;
    T& topological_scalar_attribute(const Tuple& t);

    T const_scalar_attribute(const Tuple& t) const;
    T& scalar_attribute(const Tuple& t);

    ConstMapResult const_vector_attribute(const Tuple& t) const;
    MapResult vector_attribute(const Tuple& t);


    using BaseType::dimension; // const() -> int64_t
    using BaseType::reserved_size; // const() -> int64_t

    using BaseType::attribute; // access to Attribute object being used here
    using CachingBaseType::has_stack;
    using CachingBaseType::mesh;
    using CachingBaseType::stack_depth;

protected:
    int64_t index(const Tuple& t) const;
    using CachingBaseType::base_type;
    CachingBaseType& caching_base_type() { return *this; }
    const CachingBaseType& caching_base_type() const { return *this; }

public:
    CachingBaseType& index_access() { return caching_base_type(); }
    const CachingBaseType& index_access() const { return caching_base_type(); }
};
} // namespace wmtk::attribute
#include "Accessor.hxx"
