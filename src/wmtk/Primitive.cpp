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
    constexpr size_t valid_ids = sizeof(names) / sizeof(std::string) - 1;
    if (id >= 0 && id <= valid_ids) {
        return std::string_view(names[id]);
    } else {
        return std::string_view(names[valid_ids]);
    }
}
} // namespace wmtk
