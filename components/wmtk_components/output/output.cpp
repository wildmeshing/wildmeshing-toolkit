#include "output.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include "internal/OutputOptions.hpp"

namespace wmtk::components {

void output(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    OutputOptions options = j.get<OutputOptions>();

    std::shared_ptr<Mesh> mesh = cache.read_mesh(options.input);

    std::array<bool, 4> out = {{false, false, false, false}};
    for (int64_t d = 0; d <= mesh->top_cell_dimension(); ++d) {
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