#pragma once
#include <type_traits>
#include "wmtk/Primitive.hpp"
namespace wmtk {
    class Mesh;
namespace attribute {
template <typename T>
class MeshAttributes;
template <typename T>
class AccessorBase;
template <typename T>
class TupleAccessor;
template <typename T>
class MeshAttributeHandle;
struct AttributeManager;

class AttributeHandle
{
protected:
    template <typename T>
    friend class MeshAttributes;
    template <typename T>
    friend class MeshAttributeHandle;
    friend struct AttributeManager;
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
    friend class wmtk::Mesh;
    friend class MeshAttributes<T>;
    friend class AccessorBase<T>;
    friend class TupleAccessor<T>;
    friend struct AttributeManager;
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
} // namespace attribute
using AttributeHandle = attribute::AttributeHandle;
template <typename T>
using MeshAttributeHandle = attribute::MeshAttributeHandle<T>;
} // namespace wmtk
