#pragma once
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen::utils {
auto simplex_index_from_valid_index(
    const PrimitiveType mesh_type,
    int8_t valid_index,
    wmtk::PrimitiveType type) -> int8_t;

}

