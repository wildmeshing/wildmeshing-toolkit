#pragma once
#include <cstdint>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen {
class SimplexDart;
}
namespace wmtk::autogen::utils {
/// @brief: Returns the relative local index of a simplex with respect to an orientation
/// When we switch a dart to lie on a specific simplex we need to find some action that maps to that simplex.
/// We typically encode these target simplices in terms of indices used in the identity orientation, so this function rewrites an index with repsect to the identity orientation to the given input orientation
/// @param sd the context for the type of simplex we are in
/// @param current_orientation the orientation we want to get the relative ilocal index of
/// @param target_type the type of simplex we are trying to get the relative index of
/// @param the global index of a simplex with respect to the identity dart
int8_t relative_face_index(
    const SimplexDart& sd,
    int8_t current_orientation,
    PrimitiveType target_type,
    int8_t target_index);

} // namespace wmtk::autogen::utils
