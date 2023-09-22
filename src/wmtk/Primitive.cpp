#include "Primitive.hpp"
#include <string>



namespace wmtk {
    namespace {
        const static std::string names[] = {
            "Vertex",
            "Edge",
            "Face",
            "Tetrahedron",
            "Invalid"
        };
    }

std::string_view primitive_type_name(PrimitiveType t) {
    long dim = get_simplex_dimension(t);
    if(dim >= 0 && dim <= 3) {
        return names[dim];
    } else {
        return names[4];
    }

    
}
}
