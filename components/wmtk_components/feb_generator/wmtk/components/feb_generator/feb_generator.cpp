#include "feb_generator.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>


namespace wmtk::components {

void feb_generator(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    // ExtractOptions options = j.get<ExtractOptions>();

    // std::string input_file = options.input;
    // std::string output_file = options.output;

    // if (!std::filesystem::exists(input_file)) {
    //     throw std::runtime_error(std::string("file") + input_file + " not found");
    // }

    // if (options.mode == true) {
    //     // extract_triangle_soup_from_image
    //     unsigned int level = options.level;
    //     internal::extract_triangle_soup_from_image(output_file, input_file, level);
    // } else {
    //     // gmsh2hdf_tag
    //     std::string encoded_file = options.volumetric_encoded_file;
    //     std::string encoded_bc_file = options.volumetric_encoded_bc_file;
    //     double delta_x = options.delta_x;
    //     if (encoded_bc_file == "dummy") {
    //         internal::gmsh2hdf_tag(encoded_file, input_file, output_file, delta_x);
    //     } else {
    //         internal::gmsh2hdf_tag(encoded_file, encoded_bc_file, input_file, output_file,
    //         delta_x);
    //     }
    // }
}
} // namespace wmtk::components