#pragma once
#include <cstdint>
#include <wmtk/Tuple.hpp>

namespace wmtk::autogen::utils {
int8_t valid_index_to_simplex_index(PrimitiveType mesh_type, const Tuple& t);

}
