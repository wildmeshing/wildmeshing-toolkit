#pragma once

#include "SurfaceMap.hpp"
#include "surface_io.hpp"

#include <string>

namespace wmtk::components::tracking {

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
 * face also gets `input_face` (Int64): the input face its centroid maps to, -1 for none. A face
 * field keeps its type and column names; a value with no source is 0. `warnings`, if
 * given, gets what mapping the output's points to the input found.
 */
SurfaceData
transfer_fields(const SurfaceMap& map, const SurfaceData& input, MapWarnings* warnings = nullptr);

/// The vertex of the map's output each vertex of transfer_fields' result is: the vertices of its
/// live faces, by rank.
std::vector<uint64_t> transferred_vertices(const SurfaceMap& map);

/**
 * @brief The vertex of the application's output each vertex of transfer_fields' result is, by
 * the history's output_vertex_ids: for a tet output, its rows in the tet mesh, which
 * add_surface takes. -1 for one the output does not have.
 */
std::vector<int64_t> output_rows(const SurfaceMap& map, const std::vector<uint64_t>& output_ids);

} // namespace wmtk::components::tracking
