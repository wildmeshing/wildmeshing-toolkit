#pragma once

#include <wmtk/MeshData.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/VolumeHistory.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>

namespace wmtk::components::tracking {

/**
 * @brief The tracking app's `volume`: builds the map of `volume`, recorded with `h`, and checks it
 * on `samples` points sampled near the tracked surface of `tet_mesh`, the mesh the volume history
 * starts from -- how far they move, the map's conformal distortion and Jacobian there, whether
 * they end in their region of `output_tet_mesh` (if given), the mesh it ends at, and how close
 * the inverse brings them back. Without tet_mesh, every operation's map is built. Returns what it
 * found, as it logs it: the report of both routes.
 */
nlohmann::json volume_info(
    const history::SurfaceHistory& h,
    history::VolumeHistory volume,
    const MeshData* tet_mesh,
    const MeshData* output_tet_mesh,
    size_t samples);
/// The tracking app's `map_volume` (file route only): maps the points of the file `points`.
void map_volume_points(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root);

} // namespace wmtk::components::tracking
