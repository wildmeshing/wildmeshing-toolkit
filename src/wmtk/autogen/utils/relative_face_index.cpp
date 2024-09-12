
#include "relative_face_index.hpp"
#include <wmtk/autogen/SimplexDart.hpp>

namespace wmtk::autogen {
class SimplexDart;
}
namespace wmtk::autogen::utils {
int8_t relative_face_index(
    const SimplexDart& sd,
    int8_t current_orientation,
    PrimitiveType target_type,
    int8_t target_index)
{
    const int8_t identity_action_to_right_face = 0; // sd.simplex_index(target_index, target_type);
    const int8_t action_to_right_face =
        sd.product(identity_action_to_right_face, sd.inverse(current_orientation));

    const int8_t relative_face = sd.simplex_index(action_to_right_face, target_type);
    return relative_face;
}

} // namespace wmtk::autogen::utils
