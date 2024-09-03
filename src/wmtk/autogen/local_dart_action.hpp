
#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/autogen/Dart.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
class SimplexDart;
int8_t local_dart_action(const SimplexDart& sd, int8_t s, int8_t action);
Dart local_dart_action(const SimplexDart& sd, const Dart& d, int8_t target);

} // namespace wmtk::autogen
