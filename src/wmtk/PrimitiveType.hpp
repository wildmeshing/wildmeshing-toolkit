#pragma once

#include <stdexcept>
#include <string_view>
#include <vector>

namespace wmtk {

enum class PrimitiveType : int8_t { Vertex = 0, Edge = 1, Triangle = 2, Tetrahedron = 3 };


/**
 * @brief Get a unique integer id corresponding to each primitive type
 *
 * Ordering of primitive types by dimension allows to exploit the fact that all m<n dimensional
 * primitives exist in an n dimensional manifold
 */
constexpr inline int8_t get_primitive_type_id(PrimitiveType t)
{
    return static_cast<int8_t>(t);
}

constexpr inline PrimitiveType operator-(PrimitiveType pt, int8_t n) {
    return static_cast<PrimitiveType>(static_cast<int8_t>(pt)-n);
}
constexpr inline PrimitiveType operator+(PrimitiveType pt, int8_t n) {
    return static_cast<PrimitiveType>(static_cast<int8_t>(pt)+n);
}

constexpr inline bool operator==(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) == get_primitive_type_id(b);
}
constexpr inline bool operator!=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) != get_primitive_type_id(b);
}
constexpr inline bool operator<(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) < get_primitive_type_id(b);
}
constexpr inline bool operator>(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) > get_primitive_type_id(b);
}
constexpr inline bool operator<=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) <= get_primitive_type_id(b);
}
constexpr inline bool operator>=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) >= get_primitive_type_id(b);
}


/**
 * @brief Get the primitive type corresponding to its unique integer id
 */
constexpr inline PrimitiveType get_primitive_type_from_id(int8_t id)
{
    return static_cast<PrimitiveType>(id);
}

/**
 * @brief Get the maximum id for a list of primitive types
 *
 * @param primitive_types: list of primitive types
 * @return maximum primitive type id among the list
 */
int8_t get_max_primitive_type_id(const std::vector<PrimitiveType>& primitive_types);

std::string_view primitive_type_name(PrimitiveType t);

} // namespace wmtk
