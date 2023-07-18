#pragma once
#include <type_traits>
#include "Primitive.hpp"
namespace wmtk {
template <typename T>
class MeshAttributes;
template <typename T, bool IsConst>
class Accessor;
template <typename T>
class AccessorBase;
template <typename T>
class MeshAttribteHandle;
class AttributeManager;

class AttributeHandle
{
protected:
    template <typename T>
    friend class MeshAttributes;
    template <typename T>
    friend class MeshAttributeHandle;
    friend class AttributeManager;
    long index = -1;
    AttributeHandle(long i)
        : index(i)
    {}

public:
    AttributeHandle() = default;
    AttributeHandle(const AttributeHandle&) = default;
    AttributeHandle(AttributeHandle&&) = default;
    AttributeHandle& operator=(const AttributeHandle&) = default;
    AttributeHandle& operator=(AttributeHandle&&) = default;


    bool operator==(const AttributeHandle& other) const { return index == other.index; }
};

template <typename T>
class MeshAttributeHandle
{
private:
    friend class Mesh;
    friend class MeshAttributes<T>;
    friend class AccessorBase<T>;
    friend class AttributeManager;
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;

    MeshAttributeHandle(AttributeHandle ah, PrimitiveType pt)
        : m_base_handle(ah)
        , m_primitive_type(pt)
    {}
    MeshAttributeHandle(long index, PrimitiveType pt)
        : MeshAttributeHandle(AttributeHandle(index), pt)
    {}

public:
    MeshAttributeHandle() = default;
    MeshAttributeHandle(const MeshAttributeHandle&) = default;
    MeshAttributeHandle(MeshAttributeHandle&&) = default;
    MeshAttributeHandle& operator=(const MeshAttributeHandle&) = default;
    MeshAttributeHandle& operator=(MeshAttributeHandle&&) = default;

    template <typename U>
    bool operator==(const MeshAttributeHandle& o) const
    {
        return std::is_same_v<T, U> && m_base_handle == o.m_base_handle &&
               m_primitive_type == o.m_primitive_type;
    }
};
} // namespace wmtk
