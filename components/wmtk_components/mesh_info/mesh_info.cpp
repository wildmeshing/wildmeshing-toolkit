#include "mesh_info.hpp"


#include "internal/MeshInfoOptions.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {
namespace components {
void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    MeshInfoOptions options = j.get<MeshInfoOptions>();

    const std::filesystem::path& file = files[options.input];

    std::shared_ptr<Mesh> mesh_in = read_mesh(file);

    if (mesh_in->top_simplex_type() != PrimitiveType::Face) {
        wmtk::logger().warn("Info works only for triangle meshes: {}", mesh_in->top_simplex_type());
        return;
    }


    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    const auto v_tuples = mesh.get_all(PrimitiveType::Vertex);
    const auto f_tuples = mesh.get_all(PrimitiveType::Face);

    wmtk::logger().info("Mesh: {}", file.string());
    wmtk::logger().info("Vertices: {}", v_tuples.size());
    wmtk::logger().info("Faces: {}", f_tuples.size());
}
} // namespace components
} // namespace wmtk