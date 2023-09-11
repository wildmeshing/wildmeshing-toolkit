#pragma once

#include <memory>
#include <type_traits>
#include "Attribute.hpp"
#include "AttributeHandle.hpp"
#include "wmtk/Tuple.hpp"
#include "wmtk/Types.hpp"

#include <Eigen/Dense>

namespace wmtk::attribute {

template <typename T>
class MeshAttributes;
template <typename T>
class AccessorCache;

// The basic implementation of an accessor using indices.
// This should never be externally used except within the main accessor
// interface, in unit tests, or in topological updates
template <typename _T>
class AccessorBase
{
public:
    using T = _T;
    friend class AccessorCache<T>;
    using MeshAttributesType = MeshAttributes<T>;
    using AttributeType = Attribute<T>;

    using MapResult = typename VectorX<T>::MapType;
    using ConstMapResult = typename VectorX<T>::ConstMapType;


public:
    // returns the size of the underlying attribute
    long reserved_size() const;
    long dimension() const;


    void set_attribute(std::vector<T> value);

    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    MeshAttributes<T>& attributes();
    const MeshAttributes<T>& attributes() const;

    Attribute<T>& attribute();
    const Attribute<T>& attribute() const;


    ~AccessorBase();
    AccessorBase(Mesh& m, const MeshAttributeHandle<T>& handle);

    const MeshAttributeHandle<T>& handle() const;
    PrimitiveType primitive_type() const;

    Mesh& mesh();
    const Mesh& mesh() const;


protected:
    Mesh& m_mesh;
    MeshAttributeHandle<T> m_handle;

    const AttributeManager& attribute_manager() const;
    AttributeManager& attribute_manager();
};


} // namespace wmtk::attribute
