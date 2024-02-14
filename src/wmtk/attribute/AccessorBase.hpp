#pragma once

#include <memory>
#include <type_traits>
#include "MeshAttributeHandle.hpp"
#include "wmtk/Tuple.hpp"
#include "wmtk/Types.hpp"
#include "internal/MapTypes.hpp"

#include <Eigen/Dense>

namespace wmtk {
class AttributeManager;
}
namespace wmtk::attribute {

template <typename T>
class Attribute;
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

    template <int D = Eigen::Dynamic>
    using MapResult = internal::MapResult<T,D>;
    template <int D = Eigen::Dynamic>
    using ConstMapResult = internal::ConstMapResult<T,D>;


public:
    // returns the size of the underlying attribute
    int64_t reserved_size() const;
    int64_t dimension() const;


    void set_attribute(std::vector<T> value);

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const int64_t index) const;
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(const int64_t index);

    T const_scalar_attribute(const int64_t index) const;
    T& scalar_attribute(const int64_t index);

    T const_scalar_attribute(const int64_t index, const int8_t offset) const;
    T& scalar_attribute(const int64_t index, const int8_t offset);

    MeshAttributes<T>& attributes();
    const MeshAttributes<T>& attributes() const;

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

    const AttributeManager& attribute_manager() const;
    AttributeManager& attribute_manager();
};


} // namespace wmtk::attribute
#include "AccessorBase.hxx"
