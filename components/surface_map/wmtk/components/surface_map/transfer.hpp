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
 * @brief The map's output surface, with every field of `input` carried onto it.
 *
 * An output face takes the per-face values of the input face its centroid maps to. An output
 * corner takes the per-corner values interpolated where a point just inside the face at that
 * corner maps to: inside, so that it lands in the chart of the face when the corner is on a
 * seam, which the remeshing keeps as a curve. `input` must match the map's input surface
 * (matches_input). The output's live vertices and faces are numbered by rank. Every output
 * face also gets `input_face`: the input face its centroid maps to, -1 for none. `warnings`, if
 * given, gets what mapping the output's points to the input found.
 */
SurfaceData
transfer_fields(const SurfaceMap& map, const SurfaceData& input, MapWarnings* warnings = nullptr);

} // namespace wmtk::components::surface_map
