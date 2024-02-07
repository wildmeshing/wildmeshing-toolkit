#include "get_all_meshes.hpp"


#include "GetAllMeshesOptions.hpp"

#include <wmtk/utils/Logger.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>

namespace wmtk::components {

void get_all_meshes(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    GetAllMeshesOptions options = j.get<GetAllMeshesOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    auto get_all = [&](auto&& m) {
        const std::string name =
            fmt::format("{}_{}", options.name, fmt::join(m.absolute_multi_mesh_id(), ""));
        logger().info("Exporting {}", name);
        cache.write_mesh(*m.shared_from_this(), name);
    };
    multimesh::MultiMeshVisitor visitor(get_all);
    visitor.execute_from_root(*mesh_in);
}

} // namespace wmtk::components
