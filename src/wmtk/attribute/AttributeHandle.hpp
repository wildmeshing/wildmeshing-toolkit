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

    bool is_valid() const { return index != -1; }
};

} // namespace attribute
using AttributeHandle = attribute::AttributeHandle;
} // namespace wmtk

#include "MeshAttributeHandle.hpp"
