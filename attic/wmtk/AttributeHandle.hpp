#pragma once
#include "Primitive.hpp"
namespace wmtk {

struct AttributeHandle
{
public:
    int64_t index;
    int64_t stride;
};
template <typename T>
struct MeshAttributeHandle
{
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;
};
} // namespace wmtk
