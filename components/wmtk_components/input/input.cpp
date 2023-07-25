#include "input.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/MeshUtils.hpp>

#include "internal/InputOptions.hpp"

namespace wmtk {
namespace components {
void input(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    if (!std::filesystem::exists(options.file)) {
        throw std::runtime_error(std::string("file") + options.file.string() + " not found");
    }

    TriMesh mesh;
    if (options.file.extension() == ".hdf5") {
        MeshReader reader(options.file);
        reader.read(mesh);
    } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
        Eigen::MatrixXd V;
        Eigen::Matrix<long, -1, -1> F;
        igl::read_triangle_mesh(options.file.string(), V, F);

        mesh.initialize(F);

        MeshUtils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

    } else {
        throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
    }

    const std::filesystem::path cache_dir = "cache";
    std::filesystem::create_directory(cache_dir);

    const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

    HDF5Writer writer(cached_mesh_file);
    mesh.serialize(writer);

    files[options.name] = cached_mesh_file;
}
} // namespace components
} // namespace wmtk