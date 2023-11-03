#pragma once
#include "Primitive.hpp"
namespace wmtk {

struct AttributeHandle
{
public:
    long index;
    long stride;
};
template <typename T>
struct MeshAttributeHandle
{
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;
};
} // namespace wmtk
