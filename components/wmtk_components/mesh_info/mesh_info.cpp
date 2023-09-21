#include "mesh_info.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>

namespace wmtk {
namespace components {
void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    MeshInfoOptions options = j.get<MeshInfoOptions>();

    const std::filesystem::path& file = files[options.input];

    TriMesh mesh;
    {
        MeshReader reader(file);
        reader.read(mesh);
    }

    const auto v_tuples = mesh.get_all(PrimitiveType::Vertex);
    const auto f_tuples = mesh.get_all(PrimitiveType::Face);

    wmtk::logger().info("Mesh: {}", file.string());
    wmtk::logger().info("Vertices: {}", v_tuples.size());
    wmtk::logger().info("Faces: {}", f_tuples.size());
}
} // namespace components
} // namespace wmtk