#pragma once

#include "SurfaceMap.hpp"
#include "surface_io.hpp"

#include <string>

namespace wmtk::components::surface_map {

/**
 * @brief Whether `input` is the surface the map starts from: the same faces, in the same order,
 * with the same corner positions. Explains the first difference in `why` if not.
 */
bool matches_input(const SurfaceMap& map, const SurfaceData& input, std::string& why);

/**
 * @brief The map's output surface, with every per-face field of `input` carried onto it.
 *
 * An output face takes the value of the input face its centroid maps to. `input` must match the
 * map's input surface (matches_input). The output's live vertices and faces are numbered by rank.
 */
SurfaceData transfer_face_fields(const SurfaceMap& map, const SurfaceData& input);

} // namespace wmtk::components::surface_map
