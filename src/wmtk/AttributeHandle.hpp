#pragma once
#include "Primitive.hpp"
namespace wmtk {
template <typename T>
class MeshAttributes;
template <typename T, bool IsConst>
class Accessor;
template <typename T>
class AccessorBase;

class AttributeHandle
{
protected:
    template <typename T>
    friend class MeshAttributes;
    long index = -1;

public:
    bool valid() const { return index >= 0; }

    bool operator==(const AttributeHandle& other) const { return index == other.index; }
};

template <typename T>
class MeshAttributeHandle
{
private:
    friend class Mesh;
    friend class MeshAttributes<T>;
    friend class AccessorBase<T>;
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type = PrimitiveType::Invalid;

public:
    bool valid() const
    {
        return m_base_handle.valid() && m_primitive_type != PrimitiveType::Invalid;
    }
};
} // namespace wmtk
