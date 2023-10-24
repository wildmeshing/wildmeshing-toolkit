#include "Primitive.hpp"
#include <string>


namespace wmtk {
namespace {
const static std::string names[] = {
    "Vertex",
    "Edge",
    "Face",
    "Tetrahedron",
    "HalfEdge"
    "Invalid"};
}

long get_max_primitive_type_id(const std::vector<PrimitiveType>& primitive_types)
{
    long max_id = -1;
    for (const auto& t : primitive_types) {
        max_id = std::max(max_id, get_primitive_type_id(t));
    }

    return max_id;
}

PrimitiveType get_primitive_type_from_id(long id)
{
    switch (id) {
    case 0: return PrimitiveType::Vertex;
    case 1: return PrimitiveType::Edge;
    case 2: return PrimitiveType::Face;
    case 3: return PrimitiveType::Tetrahedron;
    case 4: return PrimitiveType::HalfEdge;
    default: break; // just return at the end because compilers can be finicky
    }

    return PrimitiveType::Vertex;
}

std::string_view primitive_type_name(PrimitiveType t)
{
    long id = get_primitive_type_id(t);
    long num_ids = 5;
    if (id >= 0 && id <= num_ids) {
        return names[id];
    } else {
        return names[num_ids];
    }
}
} // namespace wmtk
