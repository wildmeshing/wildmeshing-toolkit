#pragma once

#include <string_view>

namespace wmtk {

enum class PrimitiveType { Vertex, Edge, HalfEdge, Face, Tetrahedron };

// TODO Rename to get_cell_dimension
constexpr long get_simplex_dimension(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
    case PrimitiveType::HalfEdge: {
        throw std::runtime_error("halfedge is not a simplex");
        break;
    }
    default: break; // just return at the end because compilers can be finicky
    }

    return -2;
}

constexpr bool operator==(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) == get_simplex_dimension(b);
}
constexpr bool operator!=(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) != get_simplex_dimension(b);
}
constexpr bool operator<(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) < get_simplex_dimension(b);
}
constexpr bool operator>(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) > get_simplex_dimension(b);
}
constexpr bool operator<=(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) <= get_simplex_dimension(b);
}
constexpr bool operator>=(PrimitiveType a, PrimitiveType b)
{
    return get_simplex_dimension(a) >= get_simplex_dimension(b);
}

/**
 * @brief Get a unique integer id corresponding to each primitive type
 *
 * Ordering of primitive types by dimension allows to exploit the fact that all m<n dimensional
 * primitives exist in an n dimensional manifold
 */
constexpr long get_primitive_type_id(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::HalfEdge: return 2;
    case PrimitiveType::Face: return 3;
    case PrimitiveType::Tetrahedron: return 4;
    default: break; // just return at the end because compilers can be finicky
    }

    return -2;
}

/**
 * @brief Get the number of primitive types corresponding to a given mesh dimension
 */
long get_dimension_primitive_counts(long dimension);

// TODO Make into a switch, including HalfEdge?
std::string_view primitive_type_name(PrimitiveType t);
} // namespace wmtk
