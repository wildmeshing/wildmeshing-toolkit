#include "MeshReader.hpp"
#include "HDF5Reader.hpp"

#include <memory>

namespace wmtk {

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes)
{
    const auto extension = filename.extension().string();
    if (extension == ".hdf5") {
        HDF5Reader reader;
        return reader.read(filename);
    } else if (extension == ".msh") {
        MshReader reader;
        return reader.read(filename, ignore_z, tetrahedron_attributes);
    }

    throw std::runtime_error(extension + " not supported");
}

} // namespace wmtk