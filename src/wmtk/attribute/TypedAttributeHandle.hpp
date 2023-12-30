
#pragma once
#include "AttributeHandle.hpp"
namespace wmtk {
class Mesh;
class Rational;
namespace attribute {
template <typename T>
class MeshAttributes;
template <typename T>
class AccessorBase;
template <typename T>
class TupleAccessor;
class AttributeManager;


/* @brief Handle that represents attributes for some mesh
 *
 * In conjunction with the mesh that constructed it, specifies a particular attribute.
 * NOTE: with multimesh around, you ideally want to store a
 * SmartAttributeHandle, Explicitly storing MeshAttribteHandle is
 * deprecated.
 */
template <typename T>
class TypedAttributeHandle
{
private:
    friend class wmtk::Mesh;
    friend class MeshAttributes<T>;
    friend class AccessorBase<T>;
    friend class TupleAccessor<T>;
    friend class AttributeManager;
    friend struct wmtk::hash<TypedAttributeHandle<T>>;
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;

    TypedAttributeHandle(AttributeHandle ah, PrimitiveType pt)
        : m_base_handle(ah)
        , m_primitive_type(pt)
    {}
    TypedAttributeHandle(long index, PrimitiveType pt)
        : TypedAttributeHandle(AttributeHandle(index), pt)
    {}

public:
    TypedAttributeHandle() = default;
    TypedAttributeHandle(const TypedAttributeHandle&) = default;
    TypedAttributeHandle(TypedAttributeHandle&&) = default;
    TypedAttributeHandle& operator=(const TypedAttributeHandle&) = default;
    TypedAttributeHandle& operator=(TypedAttributeHandle&&) = default;

    template <typename U>
    bool operator==(const TypedAttributeHandle<U>& o) const
    {
        return std::is_same_v<T, U> && m_base_handle == o.m_base_handle &&
               m_primitive_type == o.m_primitive_type;
    }
    bool is_valid() const { return m_base_handle.is_valid(); }
    PrimitiveType primitive_type() const { return m_primitive_type; }
    const AttributeHandle& base_handle() const { return m_base_handle; }
};
} // namespace attribute
template <typename T>
using TypedAttributeHandle = attribute::TypedAttributeHandle<T>;
} // namespace wmtk
