#pragma once

#include <memory>
#include <type_traits>
#include "wmtk/Attribute.hpp"
#include "wmtk/AttributeHandle.hpp"
#include "wmtk/Tuple.hpp"
#include "wmtk/Types.hpp"

#include <Eigen/Dense>

class DEBUG_TriMesh;
namespace wmtk {

template <typename T>
class MeshAttributes;
template <typename T>
class AccessorCache;


// The basic implementation of an accessor using indices.
// This should never be
template <typename _T>
class AccessorBase
{
public:
    using T = _T;
    friend class AccessorCache<T>;
    friend class DEBUG_TriMesh;
    using MeshAttributesType = MeshAttributes<T>;
    using AttributeType =  Attribute<T>;

    using MapResult = typename VectorX<T>::MapType;
    using ConstMapResult = typename VectorX<T>::ConstMapType;





public:
    // returns the size of the underlying attribute
    long size() const;
    long stride() const;


    void set_attribute(std::vector<T> value);

    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    MeshAttributes<T>& attributes();
    const MeshAttributes<T>& attributes() const;

    AttributeType& attribute();
    const Attribute<T>& attribute() const;

    long index(const Tuple& t) const;

    ~AccessorBase();
    AccessorBase(Mesh& m, const MeshAttributeHandle<T>& handle);

protected:
    Mesh& m_mesh;
    MeshAttributeHandle<T> m_handle;
};


} // namespace wmtk
