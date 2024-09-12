#pragma once
#include <array>
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>


namespace wmtk::autogen::utils {


int8_t largest_shared_subdart_size(
    PrimitiveType mesh_type,
    int8_t dart_index,
    PrimitiveType primitive_type,
    int8_t simplex_index);
int8_t largest_shared_subdart_size(
    PrimitiveType mesh_type,
    int8_t dart_index,
    int8_t simplex_dimension,
    int8_t simplex_index);
}
