#pragma once
#include "SimplexDart.hpp"

#include "wmtk/multimesh/utils/find_local_dart_action.hpp"
namespace wmtk::autogen {
int8_t find_local_dart_action(const SimplexDart& sd, int8_t source, int8_t target)
{
    int8_t src_inv = sd.inverse(source);
    return sd.product(target, src_inv);
}
} // namespace wmtk::autogen
