#include "feb_generator.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/components/feb_generator/internal/FEBGenerator.hpp>
#include <wmtk/components/feb_generator/internal/FEBGeneratorOptions.hpp>
#include <wmtk/io/HDF5Reader.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>


namespace wmtk::components {

void feb_generator(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    FEBGeneratorOptions options = j.get<FEBGeneratorOptions>();

    std::string input_file = options.input;
    std::string json_file = options.settings_input;
    std::string output_folder = options.output_folder;

    wmtk::HDF5Reader reader;
    auto mesh_in = reader.read(input_file);
    wmtk::TetMesh& m = static_cast<wmtk::TetMesh&>(*mesh_in);

    json jsons = wmtk::components::internal::read_json_settings(json_file);

    wmtk::components::internal::generate_feb_files(m, jsons, output_folder);
}
} // namespace wmtk::components