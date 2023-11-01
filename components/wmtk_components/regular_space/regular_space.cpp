#include "regular_space.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/RegularSpace.hpp"
#include "internal/RegularSpaceOptions.hpp"

namespace wmtk {
namespace components {
void regular_space(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    RegularSpaceOptions options = j.get<RegularSpaceOptions>();

    // input
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    int dim = options.dimension;
    MeshAttributeHandle<double> pos_handle =
        mesh.get_attribute_handle<double>(options.pos_attribute_name, PrimitiveType::Vertex);
    MeshAttributeHandle<long> vertex_tag_handle =
        mesh.get_attribute_handle<long>(options.vertex_tag_handle_name, PrimitiveType::Vertex);

    switch (dim) {
    case 0: {
        MeshAttributeHandle<long> edge_tag_handle =
            mesh.register_attribute<long>(std::string("dummy"), PrimitiveType::Edge, 1);
        RegularSpace rs(
            mesh,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            options.input_value,
            options.embedding_value,
            options.split_value);
        rs.process();
    } break;
    case 1: {
        MeshAttributeHandle<long> edge_tag_handle =
            mesh.get_attribute_handle<long>(options.edge_tag_handle_name, PrimitiveType::Edge);
        RegularSpace rs(
            mesh,
            pos_handle,
            vertex_tag_handle,
            edge_tag_handle,
            options.input_value,
            options.embedding_value,
            options.split_value);
        rs.process();
    } break;
    case 2: {
        throw std::runtime_error("Dimension 2 (face) case has not been implemented!");
    } break;
    default: throw std::runtime_error("Only dimension 0,1,2 are supported!"); break;
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
