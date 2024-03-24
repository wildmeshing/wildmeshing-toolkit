
#pragma once
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/utils/Rational.hpp>
#include "AttributeHandle.hpp"
namespace wmtk {
class Mesh;
class Rational;
namespace attribute {
template <typename T>
class MeshAttributes;
template <typename T, int Dim>
class AccessorBase;
template <typename T, typename MeshType, int Dim>
class Accessor;
namespace utils {
template <int D>
class HybridRationalAttribute;
}

class AttributeManager;

class MeshAttributeHandle;

/** @brief Handle that represents attributes for some mesh
 *
 * In conjunction with the mesh that constructed it, specifies a particular attribute.
 * NOTE: with multimesh around, you ideally want to store a
 * MeshAttribteHandle, which also hides the templating used here.
 **/
template <typename T>
class TypedAttributeHandle
{
public:
    using Type = T;

private:
    friend class wmtk::Mesh;
    friend class MeshAttributes<T>;
    template <typename U, int Dim>
    friend class AccessorBase;
    template <typename U, typename MeshType, int Dim>
    friend class Accessor;
    friend class AttributeManager;
    friend class wmtk::hash<TypedAttributeHandle<T>>;
    template <int D>
    friend class utils::HybridRationalAttribute;
    wmtk::attribute::AttributeHandle m_base_handle;
    wmtk::PrimitiveType m_primitive_type;

    TypedAttributeHandle(AttributeHandle ah, PrimitiveType pt)
        : m_base_handle(ah)
        , m_primitive_type(pt)
    {}
    TypedAttributeHandle(int64_t index, PrimitiveType pt)
        : TypedAttributeHandle(AttributeHandle(index), pt)
    {}

    TypedAttributeHandle(const MeshAttributeHandle&);

public:
    using value_type = T;
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
    bool operator<(const TypedAttributeHandle<T>& o) const;
    bool is_valid() const { return m_base_handle.is_valid(); }
    PrimitiveType primitive_type() const { return m_primitive_type; }
    const AttributeHandle& base_handle() const { return m_base_handle; }
};
} // namespace attribute
template <typename T>
using TypedAttributeHandle = attribute::TypedAttributeHandle<T>;
} // namespace wmtk
