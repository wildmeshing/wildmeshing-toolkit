#include "extract_soup.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/ExtractOptions.hpp"
#include "internal/Extract_Soup.hpp"

namespace wmtk::components {

void extract_soup(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ExtractOptions options = j.get<ExtractOptions>();

    std::string input_file = options.input;
    std::string output_file = options.output;

    if (!std::filesystem::exists(input_file)) {
        throw std::runtime_error(std::string("file") + input_file + " not found");
    }

    if (options.mode == true) {
        // extract_triangle_soup_from_image
        unsigned int level = options.level;
        internal::extract_triangle_soup_from_image(output_file, input_file, 1, level);
    } else {
        // gmsh2hdf_tag
        std::string encoded_file = options.volumetric_encoded_file;
        internal::gmsh2hdf_tag(encoded_file, input_file, output_file);
    }
}
} // namespace wmtk::components