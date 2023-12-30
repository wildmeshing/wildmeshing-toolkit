#pragma once

#include <stdexcept>
#include <string_view>
#include <vector>

namespace wmtk {

enum class PrimitiveType { Vertex, Edge, Face, Tetrahedron, HalfEdge };

/**
 * @brief Get a unique integer id corresponding to each primitive type
 *
 * Ordering of primitive types by dimension allows to exploit the fact that all m<n dimensional
 * primitives exist in an n dimensional manifold
 */
constexpr int64_t get_primitive_type_id(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
    case PrimitiveType::HalfEdge: return 4;
    default: break; // just return at the end because compilers can be finicky
    }

    return -2;
}

constexpr bool operator==(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) == get_primitive_type_id(b);
}
constexpr bool operator!=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) != get_primitive_type_id(b);
}
constexpr bool operator<(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) < get_primitive_type_id(b);
}
constexpr bool operator>(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) > get_primitive_type_id(b);
}
constexpr bool operator<=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) <= get_primitive_type_id(b);
}
constexpr bool operator>=(PrimitiveType a, PrimitiveType b)
{
    return get_primitive_type_id(a) >= get_primitive_type_id(b);
}


/**
 * @brief Get the primitive type corresponding to its unique integer id
 */
PrimitiveType get_primitive_type_from_id(int64_t id);

/**
 * @brief Get the maximum id for a list of primitive types
 *
 * @param primitive_types: list of primitive types
 * @return maximum primitive type id among the list
 */
int64_t get_max_primitive_type_id(const std::vector<PrimitiveType>& primitive_types);

std::string_view primitive_type_name(PrimitiveType t);

} // namespace wmtk
