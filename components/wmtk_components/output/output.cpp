#include "output.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include "internal/OutputOptions.hpp"

namespace wmtk::components {

void output(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    OutputOptions options = j.get<OutputOptions>();

    const std::filesystem::path& file = files[options.input];
    std::shared_ptr<Mesh> mesh = read_mesh(file);

    std::array<bool, 4> out = {{false, false, false, false}};
    for (long d = 0; d <= mesh->top_cell_dimension(); ++d) {
        out[d] = true;
    }


    if (options.file.extension().empty()) {
        ParaviewWriter writer(options.file, "vertices", *mesh, out[0], out[1], out[2], out[3]);
        mesh->serialize(writer);
    } else {
        throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
    }
}
} // namespace wmtk::components