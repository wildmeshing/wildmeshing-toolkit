#pragma once

#include <cassert>

namespace wmtk {

enum class PrimitiveType { Invalid = -1, Vertex = 0, Edge = 1, Face = 2, Tetrahedron = 3 };

constexpr long get_simplex_dimension(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
    case PrimitiveType::Invalid: return -1;
    default: break; // just return at the end because compilers can be finicky
    }

    assert(false);
    return -2;
}
} // namespace wmtk
