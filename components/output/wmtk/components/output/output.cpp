#include "output.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::output {

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const std::string& position_attr_name)
{
    wmtk::logger().info("Saving on {}", file.string());

    if (file.extension().empty()) {
        assert(!position_attr_name.empty());
        std::array<bool, 4> out = {{false, false, false, false}};
        for (int64_t d = 0; d <= mesh.top_cell_dimension(); ++d) {
            out[d] = true;
        }
        ParaviewWriter writer(file, position_attr_name, mesh, out[0], out[1], out[2], out[3]);
        mesh.serialize(writer);
    } else if (file.extension() == ".hdf5") {
        output_hdf5(mesh, file);
    } else
        throw std::runtime_error(std::string("Unknown file type: ") + file.string());
}

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const attribute::MeshAttributeHandle& position_attr)
{
    const std::string attr_name = mesh.get_attribute_name(position_attr.as<double>());
    output(mesh, file, attr_name);
}

void output_hdf5(const Mesh& mesh, const std::filesystem::path& file)
{
    HDF5Writer writer(file);
    mesh.serialize(writer);
}

} // namespace wmtk::components
