#include "share_simplex.hpp"
#include <wmtk/autogen/SimplexDart.hpp>


namespace wmtk::autogen::utils {
    bool share_simplex(PrimitiveType mesh_type, PrimitiveType simplex_type, int8_t a, int8_t b) {
    const wmtk::autogen::SimplexDart& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);

    return sd.simplex_index(a, simplex_type) == sd.simplex_index(b,simplex_type);

    }
}
