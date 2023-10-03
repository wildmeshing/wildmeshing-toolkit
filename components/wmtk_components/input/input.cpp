#include "input.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

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

    switch (options.cell_dimension) {
    case 0: {
        // point-cloud
        PointMesh mesh;
        if (options.file.extension() == ".hdf5") {
            MeshReader reader(options.file);
            reader.read(mesh);
        } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
            Eigen::MatrixXd V;
            Eigen::Matrix<long, -1, -1> F;
            igl::read_triangle_mesh(options.file.string(), V, F);

            mesh.initialize(V.rows());

            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }

        const std::filesystem::path cache_dir = "cache";
        std::filesystem::create_directory(cache_dir);

        const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.name] = cached_mesh_file;
        break;
    }
    case 1:
        // edge mesh
        throw "not implemented";
        assert(false);
        break;
    case 2: {
        // triangle mesh
        TriMesh mesh;
        if (options.file.extension() == ".hdf5") {
            MeshReader reader(options.file);
            reader.read(mesh);
        } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
            Eigen::MatrixXd V;
            Eigen::Matrix<long, -1, -1> F;
            igl::read_triangle_mesh(options.file.string(), V, F);

            mesh.initialize(F);

            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }

        const std::filesystem::path cache_dir = "cache";
        std::filesystem::create_directory(cache_dir);

        const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.name] = cached_mesh_file;
        break;
    }
    case 3:
        // tetrahedra mesh
        assert(false);
        break;
    default: assert(false); break;
    }
}
} // namespace components
} // namespace wmtk