#pragma once
#include <cstdint>

namespace wmtk::autogen {
class SimplexDart;
}
namespace wmtk::autogen::utils {
int8_t relative_face_index(
    const SimplexDart& sd,
    int8_t current_orientation,
    PrimitiveType target_type,
    int8_t target_index);

}
