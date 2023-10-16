#include "Primitive.hpp"
#include <string>


namespace wmtk {
namespace {
const static std::string names[] = {"Vertex", "Edge", "Face", "Tetrahedron", "Invalid"};
}

long get_dimension_primitive_counts(long dimension)
{
    switch (dimension) {
    case 0: return 1; // Vertex
    case 1: return 2; // Edge
    case 2: return 4; // HalfEdge and Face
    case 3: return 5; // Tet
    default: break; // just return at the end because compilers can be finicky
    }

    return -2;
}

std::string_view primitive_type_name(PrimitiveType t)
{
    long dim = get_simplex_dimension(t);
    if (dim >= 0 && dim <= 3) {
        return names[dim];
    } else {
        return names[4];
    }
}
} // namespace wmtk
