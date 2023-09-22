#include "isotropic_remeshing.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/IsotropicRemeshing.hpp"
#include "internal/IsotropicRemeshingOptions.hpp"

namespace wmtk {
namespace components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(const TriMesh& mesh, const double length_rel)
{
    auto pos_handle = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor(pos_handle);

    Eigen::Vector3d p_max;
    p_max.setConstant(std::numeric_limits<double>::lowest());
    Eigen::Vector3d p_min;
    p_max.setConstant(std::numeric_limits<double>::max());

    for (const Tuple& v : mesh.get_all(PrimitiveType::Vertex)) {
        const Eigen::Vector3d p = pos.const_vector_attribute(v);
        p_max[0] = std::max(p_max[0], p[0]);
        p_max[1] = std::max(p_max[1], p[1]);
        p_max[2] = std::max(p_max[2], p[2]);
        p_min[0] = std::min(p_min[0], p[0]);
        p_min[1] = std::min(p_min[1], p[1]);
        p_min[2] = std::min(p_min[2], p[2]);
    }

    const double diag_length = (p_max - p_min).norm();

    return length_rel / diag_length;
}

void isotropic_remeshing(
    const nlohmann::json& j,
    std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    IsotropicRemeshingOptions options = j.get<IsotropicRemeshingOptions>();

    // input
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(mesh, options.length_rel);
    }

    IsotropicRemeshing isotropicRemeshing(mesh, options.length_abs, options.lock_boundary);
    isotropicRemeshing.remeshing(options.iterations);

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
