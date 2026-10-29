#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

namespace wmtk::components::surface_map {

void surface_map(nlohmann::json json_params);
/**
 * @brief The memory route. `history` is the bytes of a history (a JSON binary value: a
 * component's report["history"]), never a file name. info returns its report. transfer takes its
 * surface from `meshes` by `input`, which it needs, and returns the output surface as "mesh",
 * the carried fields as triangle attributes; with `tet_mesh`, also a key of `meshes`, that mesh
 * with the fields on its tracked surface as "tet_mesh" (add_surface). map, whose points are
 * text files, throws.
 */
ComponentResult surface_map(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::surface_map
