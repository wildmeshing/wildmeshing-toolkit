#include "procedural.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/ProceduralOptions.hpp"
#include "internal/make_mesh.hpp"

namespace wmtk::components {

void procedural(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ProceduralOptions options = j.get<ProceduralOptions>();


    std::shared_ptr<Mesh> mesh = std::visit(
        [](auto&& op) { return wmtk::components::internal::procedural::make_mesh(op); },
        options.settings);
    // mesh = create(options);
    if (!bool(mesh)) {
        throw std::runtime_error("Did not obtain a mesh when generating a procedural one");
    }

    cache.write_mesh(*mesh, options.name);
}
} // namespace wmtk::components
