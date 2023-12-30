#include "marching.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/Marching.hpp"
#include "internal/MarchingOptions.hpp"

namespace wmtk {
namespace components {
void marching(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    MarchingOptions options = j.get<MarchingOptions>();

    // input
    const std::filesystem::path& file = files[options.input];
    std::shared_ptr<Mesh> tmp = read_mesh(file);
    TriMesh& mesh = static_cast<TriMesh&>(*tmp);

    int dim = options.dimension;
    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>(options.pos_attribute_name, PrimitiveType::Vertex);
    MeshAttributeHandle<int64_t> vertex_tag_handle =
        mesh.get_attribute_handle<int64_t>(options.vertex_tag_handle_name, PrimitiveType::Vertex);
    MeshAttributeHandle<int64_t> edge_tag_handle =
        mesh.get_attribute_handle<int64_t>(options.edge_tag_handle_name, PrimitiveType::Edge);

    switch (dim) {
    case 2: {
        MeshAttributeHandle<int64_t> face_filter_tag_handle = mesh.get_attribute_handle<int64_t>(
            options.face_filter_handle_name,
            PrimitiveType::Edge);
        Marching mc(
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            face_filter_tag_handle,
            options.input_value,
            options.embedding_value,
            options.split_value);
        mc.process(mesh);
    } break;
    case 3: {
        throw std::runtime_error("3D has not been implemented!");
    } break;
    default: throw std::runtime_error("dimension setting error!"); break;
    }

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
