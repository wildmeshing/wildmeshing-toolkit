#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/autogen/SimplexDart.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
int8_t local_dart_action(const SimplexDart& sd, int8_t s, int8_t action)
{
    return sd.product(action, s);
}
Dart local_dart_action(const SimplexDart& sd, const Dart& d, int8_t action)
{
    return {d.global_id(), local_dart_action(sd, d.local_orientation(), action)};
}

} // namespace wmtk::autogen
