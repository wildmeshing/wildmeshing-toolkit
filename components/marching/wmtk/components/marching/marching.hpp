#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>

#include "MarchingOptions.hpp"

namespace wmtk::components {

/**
 * @brief Perform maching tetrahedra/triangles.
 *
 * See MarchingOptions for details.
 */
void marching(Mesh& mesh, const MarchingOptions& options);

} // namespace wmtk::components
