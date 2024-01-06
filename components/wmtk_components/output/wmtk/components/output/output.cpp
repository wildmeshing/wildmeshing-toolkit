#include "output.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/OutputOptions.hpp"

namespace wmtk::components {

void output(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    OutputOptions options = j.get<OutputOptions>();

    std::shared_ptr<Mesh> mesh = cache.read_mesh(options.input);

    std::array<bool, 4> out = {{false, false, false, false}};
    for (int64_t d = 0; d <= mesh->top_cell_dimension(); ++d) {
        out[d] = true;
    }

    std::filesystem::path file(
        wmtk::components::base::resolve_path(options.file, paths.output_dir));


    if (file.extension().empty()) {
        wmtk::logger().info("Saving on {}", file.string());
        ParaviewWriter writer(file, "vertices", *mesh, out[0], out[1], out[2], out[3]);
        mesh->serialize(writer);
    } else {
        throw std::runtime_error(std::string("Unknown file type: ") + file.string());
    }
}
} // namespace wmtk::components