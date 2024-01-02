#include "input.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/InputOptions.hpp"

namespace wmtk::components {

void input(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    if (!std::filesystem::exists(options.file)) {
        throw std::runtime_error(std::string("file") + options.file.string() + " not found");
    }

    std::shared_ptr<Mesh> mesh = read_mesh(options.file);

    cache.write_mesh(*mesh, options.name);
}
} // namespace wmtk::components