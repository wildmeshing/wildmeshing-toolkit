#pragma once
namespace wmtk {

enum class PrimitiveType { Vertex, Edge, Face, Tetrahedron };

constexpr size_t get_simplex_dimension(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
    default: break; // just return at the end because compilers can be finicky
    }

    assert(false);
    return -1;
}

constexpr PrimitiveType get_primitive_type(size_t d)
{
    switch (d) {
    case 0: return PrimitiveType::Vertex;
    case 1: return PrimitiveType::Edge;
    case 2: return PrimitiveType::Face;
    case 3: return PrimitiveType::Tetrahedron;
    default:
        throw std::runtime_error("Invalid dimension");
        break; // just return at the end because compilers can be finicky
    }

    assert(false);
}
} // namespace wmtk
