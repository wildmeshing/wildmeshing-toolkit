#pragma once
#include <tuple>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen {

    // returns an action that maps a given dart orientation to a target simplex
    auto best_action_to_simplex(PrimitiveType mesh_type, int8_t orientation, PrimitiveType target_simplex_type, int8_t target_simplex_index) -> std::tuple<int8_t, PrimitiveType>;
}
