#pragma once

#include <string_view>

namespace wmtk {

enum class PrimitiveType { Vertex = 0, Edge = 1, Face = 2, Tetrahedron = 3 };

constexpr long get_simplex_dimension(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
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

std::string_view primitive_type_name(PrimitiveType t);
} // namespace wmtk
