#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

/**
 * @brief Perform maching tetrahedra/triangles.
 */
void marching(Mesh& mesh, const nlohmann::json& j);

} // namespace wmtk::components
