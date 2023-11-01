#include "input.hpp"

#include <wmtk/PointMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/InputOptions.hpp"

namespace wmtk::components {

void input(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    if (!std::filesystem::exists(options.file)) {
        throw std::runtime_error(std::string("file") + options.file.string() + " not found");
    }

    std::shared_ptr<Mesh> mesh = read_mesh(options.file);

    const std::filesystem::path cache_dir = "cache";
    std::filesystem::create_directory(cache_dir);

    const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

    HDF5Writer writer(cached_mesh_file);
    mesh->serialize(writer);

    files[options.name] = cached_mesh_file;
}
} // namespace wmtk::components