#include "mesh_info.hpp"


#include "internal/MeshInfoOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

void mesh_info(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    MeshInfoOptions options = j.get<MeshInfoOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = *mesh_in;

    wmtk::logger().info("The information given in this component is very limited for now. Sorry.");

    const auto v_tuples = mesh.get_all(PrimitiveType::Vertex);
    const auto f_tuples = mesh.get_all(PrimitiveType::Triangle);

    wmtk::logger().info("Mesh: {}", options.input);
    wmtk::logger().info("Vertices: {}", v_tuples.size());
    wmtk::logger().info("Faces: {}", f_tuples.size());
}

} // namespace wmtk::components