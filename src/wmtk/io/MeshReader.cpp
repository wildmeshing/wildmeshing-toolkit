#include "MeshReader.hpp"

#include <wmtk/Mesh.hpp>

#include "HDF5Reader.hpp"
#include "MshReader.hpp"

#include <memory>

namespace wmtk {

std::shared_ptr<Mesh> MeshReader::read(const std::filesystem::path& filename)
{
    std::unique_ptr<MeshReader> reader = nullptr;
    const auto extension = filename.extension().string();
    if (extension == ".hdf5") {
        reader = std::make_unique<HDF5Reader>();
    } else if (extension == ".msh") {
        reader = std::make_unique<MshReader>();
    }

    if (!reader)
        throw std::runtime_error(extension + " not supported");
    else
        return reader->read_aux(filename);
}

} // namespace wmtk