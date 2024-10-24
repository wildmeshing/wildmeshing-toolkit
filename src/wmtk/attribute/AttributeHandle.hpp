#pragma once
#include <type_traits>
#include "wmtk/PrimitiveType.hpp"

// TODO: is this abstraction still necessary? the original attempt was to have a generic index that
// avoided passing templates around, but in the end we still obtained a TypedAttributeHandle<T> and
// used variant to remove the templating + introduce multimesh

namespace wmtk {
template <typename T>
class hash;
class Mesh;
namespace attribute {
template <typename T>
class MeshAttributes;
template <typename T>
class TypedAttributeHandle;
class AttributeManager;

/** @brief Internal handle representation used by MeshAttributes
 *
 */
class AttributeHandle
{
protected:
public:
    template <typename T>
    friend class MeshAttributes;
    template <typename T>
    friend class TypedAttributeHandle;
    friend class AttributeManager;
    friend class wmtk::hash<AttributeHandle>;
    friend class Mesh;

    int64_t index = -1;
    AttributeHandle(int64_t i) noexcept
        : index(i)
    {}

public:
    AttributeHandle() = default;
    AttributeHandle(const AttributeHandle&) = default;
    AttributeHandle(AttributeHandle&&) = default;
    AttributeHandle& operator=(const AttributeHandle&) = default;
    AttributeHandle& operator=(AttributeHandle&&) = default;


    bool operator==(const AttributeHandle& other) const noexcept { return index == other.index; }
    bool operator<(const AttributeHandle& other) const noexcept { return index < other.index; }

    bool is_valid() const noexcept { return index != -1; }
};

} // namespace attribute
using AttributeHandle = attribute::AttributeHandle;
} // namespace wmtk
