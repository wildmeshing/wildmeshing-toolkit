#pragma once


#include <filesystem>

#include <nlohmann/json.hpp>


namespace wmtk {
class Mesh;
namespace components {
namespace multimesh {
class MeshCollection;
}
namespace mesh_info {

nlohmann::json mesh_info(const Mesh& mesh);
}
} // namespace components
} // namespace wmtk
