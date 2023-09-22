#include "delaunay.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/DelaunayOptions.hpp"

namespace wmtk {
namespace components {
void delaunay(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    DelaunayOptions options = j.get<DelaunayOptions>();

    // input
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    throw "not implemented";

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.output] = cached_mesh_file;
    }
}
} // namespace components
} // namespace wmtk
