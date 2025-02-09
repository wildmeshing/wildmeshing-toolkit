#pragma once

#if defined(AWFEAF)
#include <type_traits>
#include "MeshAttributeHandle.hpp"
#include "MapTypes.hpp"

#include <Eigen/Dense>

namespace wmtk {
class AttributeManager;
}
namespace wmtk::attribute {

template <typename T>
class Attribute;

// The basic implementation of an accessor using indices.
// This should never be externally used except within the main accessor
// interface, in unit tests, or in topological updates
template <typename _T, int Dim = Eigen::Dynamic>
class AccessorBase
{
public:
    using T = _T;
    using AttributeType = Attribute<T>;

    template <int D = Dim>
    using MapResult = MapResult<T, D>;
    template <int D = Dim>
    using ConstMapResult = ConstMapResult<T, D>;


public:
    // returns the size of the underlying attribute
    int64_t reserved_size() const;
    int64_t dimension() const;
    const T& default_value() const;


    void set_attribute(std::vector<T> value);

    template <int D = Dim>
    ConstMapResult<D> const_vector_attribute(const int64_t index) const;
    template <int D = Dim>
    MapResult<D> vector_attribute(const int64_t index);

    const T& const_scalar_attribute(const int64_t index) const;
    T& scalar_attribute(const int64_t index);

    const T& const_vector_single_value(const int64_t index, const int8_t offset) const;
    T& vector_single_value(const int64_t index, const int8_t offset);


    Attribute<T>& attribute();
    const Attribute<T>& attribute() const;


    ~AccessorBase();
    AccessorBase(Mesh& m, const TypedAttributeHandle<T>& handle);
    AccessorBase(const Mesh& m, const TypedAttributeHandle<T>& handle);

    MeshAttributeHandle handle() const;
    const TypedAttributeHandle<T>& typed_handle() const;
    PrimitiveType primitive_type() const;

    Mesh& mesh();
    const Mesh& mesh() const;


protected:
    TypedAttributeHandle<T> m_handle;
    Mesh& m_mesh;
    Attribute<T>& m_attribute;

};


} // namespace wmtk::attribute
#include "AccessorBase.hxx"

#else

#include "CachingAttribute.hpp"
namespace wmtk::attribute {
template <typename _T, int Dim = Eigen::Dynamic>
using AccessorBase = CachingAttribute<_T>&;

}

#endif
