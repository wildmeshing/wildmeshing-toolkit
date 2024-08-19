#pragma once
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen::utils {
int8_t valid_index_to_simplex_index(
    int8_t valid_index,
    PrimitiveType target_type,
    int8_t target_index);

}
