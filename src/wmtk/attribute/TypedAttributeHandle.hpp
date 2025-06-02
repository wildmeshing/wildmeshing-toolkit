
#pragma once
#include <wmtk/PrimitiveType.hpp>
namespace wmtk {
class Mesh;
class Rational;
namespace attribute {
template <typename T>
class TypedAttributeManager;
template <typename T, typename MeshType, typename AttributeType, int Dim>
class Accessor;

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
    friend class TypedAttributeManager<T>;
    template <typename U, typename MeshType, typename AttributeType, int Dim>
    friend class Accessor;
    friend class AttributeManager;

    int64_t m_index = -1;
    wmtk::PrimitiveType m_primitive_type = wmtk::PrimitiveType::Vertex;

    /**
     * @brief Constructor that takes an index.
     *
     * Set to private to ensure that handles are not created manually.
     */
    TypedAttributeHandle(int64_t index, PrimitiveType pt)
        : m_index(index)
        , m_primitive_type(pt)
    {}

    TypedAttributeHandle(const MeshAttributeHandle&);

public:
    TypedAttributeHandle() = default;
    TypedAttributeHandle(const TypedAttributeHandle&) = default;
    TypedAttributeHandle(TypedAttributeHandle&&) = default;
    TypedAttributeHandle& operator=(const TypedAttributeHandle&) = default;
    TypedAttributeHandle& operator=(TypedAttributeHandle&&) = default;

    /**
     * @brief Check if two handles reference the same attribute.
     */
    template <typename U>
    bool operator==(const TypedAttributeHandle<U>& o) const;

    bool operator<(const TypedAttributeHandle<T>& o) const;

    /**
     * @brief Check if handle holds a reasonable index.
     *
     * This check does not guarantee that the referenced attribute also exists!
     */
    bool is_valid() const { return m_index != -1; }

    PrimitiveType primitive_type() const { return m_primitive_type; }

    const int64_t& base_handle() const { return m_index; }

    operator std::string() const;
};

template <typename T>
template <typename U>
inline bool TypedAttributeHandle<T>::operator==(const TypedAttributeHandle<U>& o) const
{
    return std::is_same_v<T, U> && m_index == o.m_index && m_primitive_type == o.m_primitive_type;
}

} // namespace attribute

template <typename T>
using TypedAttributeHandle = attribute::TypedAttributeHandle<T>;

} // namespace wmtk
