#include "input.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/InputOptions.hpp"

namespace wmtk::components {

void input(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    std::string file = wmtk::components::base::resolve_path(options.file, paths.root_path);

    if (!std::filesystem::exists(file)) {
        throw std::runtime_error(std::string("file") + file + " not found");
    }

    std::shared_ptr<Mesh> mesh = read_mesh(file, options.ignore_z);

    cache.write_mesh(*mesh, options.name);
}
} // namespace wmtk::components