#pragma once

#include <memory>
#include <type_traits>
#include "Attribute.hpp"
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {

template <typename T>
class MeshAttributes;
template <typename T>
class AccessorCache;


// The basic implementation of an accessor using indices.
// This should never be
template <typename T, bool IsConst = false>
class AccessorBase
{
public:
    friend class AccessorCache<T>;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>;
    using MeshAttributesType =
        std::conditional_t<IsConst, const MeshAttributes<T>, MeshAttributes<T>>;
    using AttributeType = Attribute<T>;

    using AttributeT = std::conditional_t<IsConst, const Attribute<T>, Attribute<T>>;

    using MapResult = typename AttributeType::MapResult;
    using ConstMapResult = typename AttributeType::ConstMapResult;


    using MapResultT = std::conditional_t<IsConst, ConstMapResult, MapResult>;
    using TT = std::conditional_t<IsConst, T, T&>;


public:
    // returns the size of the underlying attribute
    long size() const;
    long stride() const;

protected:
    AccessorBase(MeshType& m, const MeshAttributeHandle<T>& handle);
    ~AccessorBase();

    void set_attribute(std::vector<T> value);


    ConstMapResult const_vector_attribute(const long index) const;
    T const_scalar_attribute(const long index) const;

    ConstMapResult vector_attribute(const long index) const;
    MapResultT vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    TT scalar_attribute(const long index);

    MeshAttributesType& attributes();
    const MeshAttributesType& attributes() const;

    AttributeT attribute();
    const Attribute<T>& attribute() const;

    long index(const Tuple& t) const;

    MeshType& m_mesh;
    MeshAttributeHandle<T> m_handle;
};


} // namespace wmtk
