#pragma once
#include "Primitive.hpp"
namespace wmtk {

struct AttributeHandle
{
public:
    long index = -1;
    long stride = -1;

    bool valid() const { return index >= 0 && stride >= 0; }

    bool operator==(const AttributeHandle& other) const
    {
        return index == other.index && stride == other.stride;
    }
};

template <typename T>
struct MeshAttributeHandle
{
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type = PrimitiveType::Invalid;

    bool valid() const
    {
        return m_base_handle.valid() && m_primitive_type != PrimitiveType::Invalid;
    }
};
} // namespace wmtk
