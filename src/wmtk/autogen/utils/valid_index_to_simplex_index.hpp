#pragma once
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen::utils {
int8_t valid_index_to_simplex_index(
    PrimitiveType mesh_type,
    int8_t valid_index,
    PrimitiveType target_type);

}
