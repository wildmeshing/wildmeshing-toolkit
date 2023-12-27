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

    if (tmp->top_simplex_type() != PrimitiveType::Face) {
        throw std::runtime_error("Marching is only implemented for triangle meshes.");
    }

    TriMesh& mesh = static_cast<TriMesh&>(*tmp);

    const auto& [input_tag_attr_name, input_tag_value_1, input_tag_value_2] = options.input_tags;

    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>(input_tag_attr_name, PrimitiveType::Vertex);

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> edge_filter_tags;
    for (const auto& [name, value] : options.edge_filter_tags) {
        MeshAttributeHandle<long> handle =
            mesh.get_attribute_handle<long>(name, PrimitiveType::Edge);
        edge_filter_tags.emplace_back(std::make_tuple(handle, value));
    }

    switch (mesh.top_cell_dimension()) {
    case 2: {
        std::tuple<MeshAttributeHandle<long>, long, long> vertex_tags =
            std::make_tuple(vertex_tag_handle, input_tag_value_1, input_tag_value_2);

        Marching mc(mesh, vertex_tags, options.output_vertex_tag, edge_filter_tags);
        mc.process();
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
