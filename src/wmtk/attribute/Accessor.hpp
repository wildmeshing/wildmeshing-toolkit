#pragma once
#include <algorithm>
#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include "CachingAttribute.hpp"
#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {
/**
 * A CachingAccessor that uses tuples for accessing attributes instead of indices.
 * As global simplex ids should not be publicly available, this accessor uses the Mesh.id() function
 * to map from a tuple to the global simplex id.
 */
template <typename T, typename MeshType = Mesh, typename AttributeType_ = CachingAttribute<T>, int Dim = Eigen::Dynamic>
class Accessor
{
public:
    friend class wmtk::Mesh;
    using Scalar = T;

    using AttributeType = AttributeType_;//CachingAttribute<T>;
    // using CachingBaseType = CachingAccessor<T, Dim>;

    // Eigen::Map<VectorX<T>>
    template <int D = Dim>
    using MapResult = typename AttributeType::template MapResult<D>;
    // Eigen::Map<const VectorX<T>>
    template <int D = Dim>
    using ConstMapResult = typename AttributeType::template ConstMapResult<D>;


    Accessor(MeshType& m, const TypedAttributeHandle<T>& handle);
    Accessor(const MeshType& m, const TypedAttributeHandle<T>& handle);

    template <typename OMType, typename OAT, int D>
    Accessor(const Accessor<T, OMType, OAT, D>& o);
    auto dimension() const -> int64_t;
    auto reserved_size() const -> int64_t;
    auto default_value() const -> const Scalar&;


    AttributeType& attribute();
    const AttributeType& attribute() const;
    // =============
    // Access methods
    // =============
    // NOTE: If any compilation warnings occur check that there is an overload for an index method

    const T& const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt) const;
    // template <typename ArgType>
    // Scalar& topological_scalar_attribute(const ArgType& t);
    template <typename ArgType>
    const Scalar& const_scalar_attribute(const ArgType& t) const;
    template <typename ArgType>
    Scalar& scalar_attribute(const ArgType& t);

    template <int D = Dim, typename ArgType = wmtk::Tuple>
    ConstMapResult<std::max(D, Dim)> const_vector_attribute(const ArgType& t) const;
    template <int D = Dim, typename ArgType = wmtk::Tuple>
    MapResult<std::max(D, Dim)> vector_attribute(const ArgType& t);


    MeshType& mesh();
    const MeshType& mesh() const;

protected:
    int64_t index(const Tuple& t) const;
    int64_t index(const simplex::Simplex& t) const;
    int64_t index(const simplex::IdSimplex& t) const;

public:
    AttributeType& index_access() { return attribute(); }
    const AttributeType& index_access() const { return attribute(); }
    TypedAttributeHandle<T> m_handle;
    MeshType& m_mesh;
    AttributeType& m_attribute;
    MeshAttributeHandle handle() const;
    const TypedAttributeHandle<T>& typed_handle() const;
    PrimitiveType primitive_type() const;
};
} // namespace wmtk::attribute
#include "Accessor.hxx"
