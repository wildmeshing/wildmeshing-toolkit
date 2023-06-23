#pragma once
#include "Primitive.hpp"
namespace wmtk {

class AttributeHandle
{
public:
    long index;
    long stride;
};
template <typename T>
class MeshAttributeHandle
{
    AttributeHandle m_base_handle;
    PrimitiveType m_primitive_type;
};
} // namespace wmtk
