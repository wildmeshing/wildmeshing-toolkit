#pragma once

#include <wmtk/history/History.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>

namespace wmtk::components::tracking {

/// The tracking app's operations on a volume history (volume_history): `volume` builds the map
/// and checks it on points sampled near the tracked surface of tet_mesh; `map_volume` maps the
/// points of a file.
void volume_info(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root);
void map_volume_points(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root);

} // namespace wmtk::components::tracking
