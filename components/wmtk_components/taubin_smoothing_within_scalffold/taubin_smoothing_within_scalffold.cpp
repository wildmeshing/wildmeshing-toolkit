#include "taubin_smoothing_within_scalffold.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>


#include <wmtk/utils/Logger.hpp>

#include "internal/TaubinSmoothingWithinScalffold.hpp"
#include "internal/TaubinSmoothingWithinScalffoldOptions.hpp"

namespace wmtk {
namespace components {
void isotropic_remeshing(
    const nlohmann::json& j,
    std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    // IsotropicRemeshingOptions options = j.get<IsotropicRemeshingOptions>();

    // // input

    // const std::filesystem::path& file = files[options.input];
    // std::shared_ptr<Mesh> mesh_in = read_mesh(file);

    // if (mesh_in->top_simplex_type() != PrimitiveType::Face) {
    //     log_and_throw_error("Info works only for triangle meshes: {}",
    //     mesh_in->top_simplex_type());
    // }

    // TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);


    // if (options.length_abs < 0) {
    //     if (options.length_rel < 0) {
    //         throw std::runtime_error("Either absolute or relative length must be set!");
    //     }
    //     options.length_abs = relative_to_absolute_length(mesh, options.length_rel);
    // }

    // IsotropicRemeshing isotropicRemeshing(mesh, options.length_abs, options.lock_boundary);
    // isotropicRemeshing.remeshing(options.iterations);

    // // output
    // {
    //     const std::filesystem::path cache_dir = "cache";
    //     const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

    //     HDF5Writer writer(cached_mesh_file);
    //     mesh.serialize(writer);

    //     files[options.output] = cached_mesh_file;
    // }
}
} // namespace components
} // namespace wmtk
