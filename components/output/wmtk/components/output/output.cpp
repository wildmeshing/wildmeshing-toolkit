#include "output.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components {

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const std::string& position_attr_name)
{
    using namespace internal;

    wmtk::logger().info("Saving on {}", file.string());

    if (file.extension().empty()) {
        std::array<bool, 4> out = {{false, false, false, false}};
        for (int64_t d = 0; d <= mesh.top_cell_dimension(); ++d) {
            out[d] = true;
        }
        ParaviewWriter writer(file, position_attr_name, mesh, out[0], out[1], out[2], out[3]);
        mesh.serialize(writer);
    } else if (file.extension() == ".hdf5") {
        HDF5Writer writer(file);
        mesh.serialize(writer);
    } else
        throw std::runtime_error(std::string("Unknown file type: ") + file.string());
}
} // namespace wmtk::components
