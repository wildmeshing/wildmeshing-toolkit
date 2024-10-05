#pragma once
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>
#include <array>

namespace wmtk::operations::internal {

int8_t left_ear_action(PrimitiveType mesh_dimension);
int8_t right_ear_action(PrimitiveType mesh_dimension);

int8_t ear_action(PrimitiveType mesh_dimension, bool is_left);
std::array<int8_t,2> ear_actions(PrimitiveType mesh_dimension);
} // namespace wmtk::operations::internal
