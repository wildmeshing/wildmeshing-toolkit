#pragma once

#include <memory>
#include <type_traits>
#include "Attribute.hpp"
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

class DEBUG_TriMesh;
namespace wmtk {

template <typename T>
class MeshAttributes;
template <typename T>
class AccessorCache;



// The basic implementation of an accessor using indices.
// This should never be
template <typename _T, bool IsConst = false>
class AccessorBase
{
public:
    using T = _T;
    friend class AccessorCache<T>;
    friend class DEBUG_TriMesh;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>;
    using MeshAttributesType =
        std::conditional_t<IsConst, const MeshAttributes<T>, MeshAttributes<T>>;

    using AttributeType = std::conditional_t<IsConst, const Attribute<T>&, Attribute<T>>;

    using MapResult = typename Attribute<T>::MapResult;
    using ConstMapResult = typename Attribute<T>::ConstMapResult;


    using MapResultT = std::conditional_t<IsConst, ConstMapResult, MapResult>;
    using TT = std::conditional_t<IsConst, T, T&>;


public:
    // returns the size of the underlying attribute
    long size() const;
    long stride() const;


    void set_attribute(std::vector<T> value);


    ConstMapResult const_vector_attribute(const long index) const;
    T const_scalar_attribute(const long index) const;

    ConstMapResult vector_attribute(const long index) const;
    MapResultT vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    TT scalar_attribute(const long index);

    MeshAttributesType& attributes();
    const MeshAttributesType& attributes() const;

    AttributeType& attribute();
    const Attribute<T>& attribute() const;

    long index(const Tuple& t) const;

    ~AccessorBase();
    AccessorBase(MeshType& m, const MeshAttributeHandle<T>& handle);
protected:

    MeshType& m_mesh;
    MeshAttributeHandle<T> m_handle;
};


} // namespace wmtk
