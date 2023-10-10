#include "isosurface_extraction.hpp"

#include <igl/adjacency_list.h>
#include <igl/avg_edge_length.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/IsosurfaceExtraction.hpp"
#include "internal/IsosurfaceExtractionOptions.hpp"

namespace wmtk {
namespace components {
// compute the length relative to the bounding box diagonal

double get_avg_length(const TriMesh& mesh)
{
    auto pos_handle = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor(pos_handle);
    double avg_length = 0.0;
    auto edges = mesh.get_all(PrimitiveType::Edge);
    for (const Tuple& e : edges) {
        const Eigen::Vector3d p0 = pos.const_vector_attribute(e);
        const Eigen::Vector3d p1 = pos.const_vector_attribute(mesh.switch_vertex(e));
        avg_length += (p0 - p1).norm() / edges.size();
    }

    return avg_length;
}

void isosurface_extraction(
    const nlohmann::json& j,
    std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    IsosurfaceExtractionOptions options = j.get<IsosurfaceExtractionOptions>();

    // input
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    if (options.scalar_field_tag_type != "long") {
        throw std::runtime_error("Currently, we only support LONG type scalar field!");
    }

    double avg_length = get_avg_length(mesh);
    if (options.inflate_abs < 0) {
        if (options.inflate_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.inflate_abs = avg_length * options.inflate_rel;
        // options.inflate_abs = relative_to_absolute_length(V, F, options.inflate_rel);
    }

    IsosurfaceExtraction iso_ex(
        mesh,
        avg_length,
        options.lock_boundary,
        options.input_tag_value,
        options.embedding_tag_value,
        options.offset_tag_value,
        options.inflate_abs);
    iso_ex.process(options.iteration_times);

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        //const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");
        const std::filesystem::path cached_mesh_file = files["output"];

        // HDF5Writer writer(cached_mesh_file);
        // mesh.serialize(writer);

        files[options.output] = cached_mesh_file;

        // for test
        ParaviewWriter writer1(cached_mesh_file, "position", mesh, true, true, true, false);
        mesh.serialize(writer1);
    }
}
} // namespace components
} // namespace wmtk