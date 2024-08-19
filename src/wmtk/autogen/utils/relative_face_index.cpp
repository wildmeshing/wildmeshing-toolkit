
#include "relative_face_index.hpp"
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/simplex_index_from_valid_index.hpp>

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
    const int8_t identity_action_to_right_face = simplex_index_from_valid_index(target_type, target_index);
    const int8_t action_to_right_face =
        sd.product(identity_action_to_right_face, sd.inverse(current_orientation()));

    const int8_t relative_face = sd.local_simplex_index(target_type, action_to_right_face);
    return relative_face;
}

} // namespace wmtk::autogen::utils
