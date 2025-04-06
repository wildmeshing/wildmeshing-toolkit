#pragma once
#include <cstdint>
#include <type_traits>

// TODO: is this abstraction still necessary? the original attempt was to have a generic index that
// avoided passing templates around, but in the end we still obtained a TypedAttributeHandle<T> and
// used variant to remove the templating + introduce multimesh

namespace wmtk {
namespace attribute {
template <typename T>
class TypedAttributeManager;
template <typename T>
class TypedAttributeHandle;
class AttributeManager;

/** @brief Internal handle representation used by TypedAttributeManager
 *
 */
class AttributeHandle
{
protected:
private:
    template <typename T>
    friend class TypedAttributeManager;
    template <typename T>
    friend class TypedAttributeHandle;
    friend class AttributeManager;

    // Index of the attribute in the TypedAttributeHandle
    int64_t m_index = -1;
    AttributeHandle(int64_t i) noexcept
        : m_index(i)
    {}
    AttributeHandle& operator=(int64_t i)
    {
        m_index = i;
        return *this;
    }


public:
    AttributeHandle() = default;
    AttributeHandle(const AttributeHandle&) = default;
    AttributeHandle(AttributeHandle&&) = default;
    AttributeHandle& operator=(const AttributeHandle&) = default;
    AttributeHandle& operator=(AttributeHandle&&) = default;


    bool operator==(const AttributeHandle& other) const noexcept
    {
        return m_index == other.m_index;
    }
    bool operator<(const AttributeHandle& other) const noexcept { return m_index < other.m_index; }

    int64_t index() const noexcept { return m_index; }
    bool is_valid() const noexcept { return m_index != -1; }
};

} // namespace attribute
using AttributeHandle = attribute::AttributeHandle;
} // namespace wmtk
